/**
 * main.c - Loop principal do simulador de controle de altitude
 *
 * Arquitetura estilo sistema embarcado:
 *   - Loop de controle determinístico com período fixo (DT = 10 ms)
 *   - Separação clara: leitura de sensor → rate limiter → pouso suave → PID → física → log
 *   - Troca de setpoint com rate limiter (rampa suave)
 *   - Reset do integrador na troca de setpoint
 *   - Falha de sensor simulada em janela de tempo específica
 *   - Modo pouso suave: limita velocidade vertical perto do solo
 *   - Saída impressa a cada 100 ms (a cada PRINT_EVERY iterações)
 *
 * Parâmetros da aeronave:
 *   - Massa: 500 kg
 *   - Empuxo máx: 10000 N  (~2× peso)
 *   - Empuxo mín: 0 N      (motor desligado)
 *
 * Parâmetros PID:
 *   - Kp=120, Ki=20, Kd=350
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pid.h"
#include "aircraft.h"

/* ------------------------------------------------------------------ */
/* Constantes de simulação                                             */
/* ------------------------------------------------------------------ */
#define DT              0.01        /* Período do loop [s] (10 ms)      */
#define SIM_DURATION    40.0        /* Duração total da simulação [s]   */
#define PRINT_EVERY     10          /* Imprime a cada N iterações        */

#define AIRCRAFT_MASS   500.0       /* Massa [kg]                        */
#define THRUST_MAX      10000.0     /* Empuxo máximo [N]                 */
#define THRUST_MIN      0.0         /* Empuxo mínimo [N]                 */
#define INITIAL_ALT     0.0         /* Altitude inicial [m]              */

/* Ganhos PID */
#define PID_KP          120.0
#define PID_KI          20.0
#define PID_KD          350.0

/* ------------------------------------------------------------------ */
/* Rate Limiter                                                        */
/* Limita a taxa de variação do setpoint para evitar degraus bruscos. */
/* A aeronave "segue" o alvo gradualmente em vez de cair em queda     */
/* livre quando o setpoint cai muito abaixo da altitude atual.        */
/* ------------------------------------------------------------------ */
#define SETPOINT_RATE_MAX   8.0     /* Máximo de variação [m/s]          */

/* ------------------------------------------------------------------ */
/* Modo Pouso Suave                                                    */
/* Abaixo de LANDING_ALT, a velocidade de descida é limitada a        */
/* LANDING_VMAX m/s. O PID passa a controlar velocidade em vez de     */
/* altitude, garantindo um pouso seguro.                              */
/* ------------------------------------------------------------------ */
#define LANDING_ALT         20.0    /* Altitude de ativação [m]          */
#define LANDING_VMAX        -2.0    /* Velocidade máxima de descida [m/s]*/

/* ------------------------------------------------------------------ */
/* Tabela de eventos da simulação                                      */
/* ------------------------------------------------------------------ */
typedef struct {
    double time;
    double setpoint;
} SetpointEvent;

static const SetpointEvent SETPOINT_SCHEDULE[] = {
    {  0.0, 100.0 },   /* t=0s:  subir para 100 m  */
    { 10.0, 200.0 },   /* t=10s: subir para 200 m  */
    { 20.0,  50.0 },   /* t=20s: descer para 50 m  */
};
#define SETPOINT_COUNT  (int)(sizeof(SETPOINT_SCHEDULE) / sizeof(SETPOINT_SCHEDULE[0]))

/* Janelas de falha de sensor */
#define FAULT_START     14.0
#define FAULT_END       16.0
#define FAULT_VALUE     999.0

/* ------------------------------------------------------------------ */
/* Funções auxiliares                                                  */
/* ------------------------------------------------------------------ */

static double get_setpoint(double t)
{
    double sp = SETPOINT_SCHEDULE[0].setpoint;
    int i;
    for (i = 0; i < SETPOINT_COUNT; i++) {
        if (t >= SETPOINT_SCHEDULE[i].time) {
            sp = SETPOINT_SCHEDULE[i].setpoint;
        }
    }
    return sp;
}

/**
 * apply_rate_limiter - Aplica rampa suave ao setpoint.
 *
 * Em vez de o setpoint pular instantaneamente de A para B, ele se move
 * no máximo SETPOINT_RATE_MAX metros por segundo. Isso evita que o PID
 * receba um erro enorme de uma vez, que causaria queda livre prolongada.
 *
 * @current : setpoint atual (rampado)
 * @target  : setpoint desejado (da tabela de eventos)
 * @dt      : período do loop
 */
static double apply_rate_limiter(double current, double target, double dt)
{
    double delta = target - current;
    double max_step = SETPOINT_RATE_MAX * dt;

    if (delta > max_step)  return current + max_step;
    if (delta < -max_step) return current - max_step;
    return target;
}

/**
 * apply_landing_mode - Modo pouso suave.
 *
 * Quando a aeronave está descendo abaixo de LANDING_ALT metros e
 * a velocidade vertical é mais negativa que LANDING_VMAX, o setpoint
 * é ajustado para forçar uma desaceleração segura.
 *
 * Estratégia: se a velocidade de descida for excessiva perto do solo,
 * seta o setpoint para a altitude atual + margem, forçando o PID a
 * frear a aeronave antes de tocar o solo.
 *
 * @sp       : setpoint rampado atual
 * @altitude : altitude real da aeronave
 * @velocity : velocidade vertical atual
 */
static double apply_landing_mode(double sp, double altitude, double velocity)
{
    /* Só ativa se estiver descendo (velocity < 0) e abaixo da altitude crítica */
    if (altitude < LANDING_ALT && velocity < LANDING_VMAX) {
        /* Força setpoint acima da posição atual para frear a descida.
         * O offset é proporcional à velocidade excessiva — quanto mais
         * rápido cai, mais agressivo o freio. */
        double excess_speed = velocity - LANDING_VMAX;  /* negativo */
        double brake_offset = -excess_speed * 3.0;      /* positivo */
        return altitude + brake_offset;
    }
    return sp;
}

static void update_sensor_fault(AltitudeSensor *sensor, double t)
{
    if (t >= FAULT_START && t < FAULT_END) {
        sensor_inject_fault(sensor, 1, FAULT_VALUE);
    } else {
        sensor_inject_fault(sensor, 0, 0.0);
    }
}

static void print_header(void)
{
    printf("%-8s  %-10s  %-10s  %-10s  %-10s  %-10s  %-12s  %s\n",
           "Tempo(s)",
           "Alt_real(m)",
           "Alt_sensor(m)",
           "Veloc(m/s)",
           "Empuxo(N)",
           "Erro(m)",
           "SP_ramp(m)",
           "Modo");
    printf("%-8s  %-10s  %-10s  %-10s  %-10s  %-10s  %-12s  %s\n",
           "--------",
           "----------",
           "-------------",
           "----------",
           "----------",
           "----------",
           "------------",
           "----");
}

static void print_state(double t, const AircraftState *ac,
                        double sensor_alt, double sp_ramp)

{
    double error = sp_ramp - sensor_alt;

    /* Determina modo ativo para exibição */
    const char *mode = "NORMAL";
    if (ac->altitude < LANDING_ALT && ac->velocity < LANDING_VMAX) {
        mode = "POUSO";
    } else if (t >= FAULT_START && t < FAULT_END) {
        mode = "FALHA_SNS";
    }

    printf("%8.2f  %10.3f  %13.3f  %10.3f  %10.2f  %10.3f  %12.3f  %s\n",
           t,
           ac->altitude,
           sensor_alt,
           ac->velocity,
           ac->thrust,
           error,
           sp_ramp,
           mode);
}

/* ------------------------------------------------------------------ */
/* Loop principal                                                      */
/* ------------------------------------------------------------------ */

int main(void)
{
    AircraftState aircraft;
    aircraft_init(&aircraft, INITIAL_ALT, AIRCRAFT_MASS);

    AltitudeSensor sensor;
    sensor_init(&sensor);

    PIDController pid;
    pid_init(&pid, PID_KP, PID_KI, PID_KD, THRUST_MIN, THRUST_MAX, DT);

    printf("\n=== SIMULADOR DE CONTROLE DE ALTITUDE (PID) ===\n");
    printf("Massa: %.0f kg | Empuxo: [%.0f, %.0f] N | dt: %.0f ms\n",
           AIRCRAFT_MASS, THRUST_MIN, THRUST_MAX, DT * 1000.0);
    printf("Rate limiter: %.1f m/s | Pouso suave: abaixo de %.0f m (vmax=%.1f m/s)\n\n",
           SETPOINT_RATE_MAX, LANDING_ALT, LANDING_VMAX);
    print_header();

    int    total_steps    = (int)(SIM_DURATION / DT);
    int    step;
    int    print_counter  = 0;
    double t              = 0.0;
    double target_sp      = get_setpoint(0.0);  /* Setpoint alvo da tabela  */
    double ramp_sp        = get_setpoint(0.0);  /* Setpoint rampado (suave) */
    double prev_target_sp = target_sp;          /* Para detectar troca      */

    for (step = 0; step <= total_steps; step++) {
        t = step * DT;

        /* 1. Falha de sensor */
        update_sensor_fault(&sensor, t);

        /* 2. Leitura do sensor */
        double alt_measured = sensor_read(&sensor, aircraft.altitude);

        /* 3. Setpoint alvo da tabela de eventos */
        target_sp = get_setpoint(t);

        /* 4. Reset do integrador na troca de setpoint */
        if (target_sp != prev_target_sp) {
            pid_reset(&pid);
            prev_target_sp = target_sp;
        }

        /* 5. Rate limiter: move o setpoint rampado em direção ao alvo
         *    no máximo SETPOINT_RATE_MAX m/s                           */
        ramp_sp = apply_rate_limiter(ramp_sp, target_sp, DT);

        /* 6. Modo pouso suave: ajusta setpoint rampado se estiver
         *    descendo muito rápido perto do solo                        */
        double pid_sp = apply_landing_mode(ramp_sp, aircraft.altitude, aircraft.velocity);

        /* 7. Computa PID com o setpoint final */
        double thrust = pid_compute(&pid, pid_sp, alt_measured);

        /* 8. Atualiza física */
        aircraft_update(&aircraft, thrust, DT);

        /* 9. Log a cada PRINT_EVERY iterações */
        if (print_counter == 0) {
            print_state(t, &aircraft, alt_measured, pid_sp);
        }
        print_counter = (print_counter + 1) % PRINT_EVERY;
    }

    printf("\n=== Simulação concluída: %.1f s ===\n", SIM_DURATION);
    printf("Altitude final: %.3f m | Velocidade final: %.3f m/s\n",
           aircraft.altitude, aircraft.velocity);

    return EXIT_SUCCESS;
}