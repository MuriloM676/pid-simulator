/**
 * main.c - Loop principal do simulador de controle de altitude
 *
 * Arquitetura estilo sistema embarcado:
 *   - Loop de controle determinístico com período fixo (DT = 10 ms)
 *   - Separação clara: leitura de sensor → PID → física → log
 *   - Troca de setpoint durante a simulação
 *   - Falha de sensor simulada em janela de tempo específica
 *   - Saída impressa a cada 100 ms (a cada PRINT_EVERY iterações)
 *
 * Parâmetros da aeronave:
 *   - Massa: 500 kg
 *   - Empuxo máx: 10000 N  (~2× peso)
 *   - Empuxo mín: 0 N      (motor desligado)
 *
 * Parâmetros PID (ajustados para esta planta):
 *   - Kp = 200, Ki = 30, Kd = 150
 */

#include <stdio.h>
#include <stdlib.h>
#include "pid.h"
#include "aircraft.h"

/* ------------------------------------------------------------------ */
/* Constantes de simulação                                             */
/* ------------------------------------------------------------------ */
#define DT              0.01        /* Período do loop [s] (10 ms)     */
#define SIM_DURATION    40.0        /* Duração total da simulação [s]  */
#define PRINT_EVERY     10          /* Imprime a cada N iterações       */

#define AIRCRAFT_MASS   500.0       /* Massa [kg]                       */
#define THRUST_MAX      10000.0     /* Empuxo máximo [N]                */
#define THRUST_MIN      0.0         /* Empuxo mínimo [N]                */
#define INITIAL_ALT     0.0         /* Altitude inicial [m]             */

/* Ganhos PID (ajuste v2)
 * Problemas anteriores:
 *   - Kp=200 muito agressivo → overshoot de 25% na fase 1
 *   - Kd=150 insuficiente para frear 500 kg em degraus grandes
 *   - Descida para 50m não convergiu em 30s
 * Solução:
 *   - Kp reduzido: menos força proporcional → menos overshoot
 *   - Kd aumentado: mais amortecimento → freia antes de ultrapassar
 *   - Ki reduzido levemente: menos acúmulo durante saturação prolongada
 */
#define PID_KP          120.0
#define PID_KI          20.0
#define PID_KD          350.0

/* ------------------------------------------------------------------ */
/* Tabela de eventos da simulação                                      */
/* Cada entrada define: [tempo, novo_setpoint ou evento especial]      */
/* ------------------------------------------------------------------ */
typedef struct {
    double time;
    double setpoint;
} SetpointEvent;

/* Troca de setpoints ao longo da simulação */
static const SetpointEvent SETPOINT_SCHEDULE[] = {
    {  0.0, 100.0 },   /* t=0s:  subir para 100 m  */
    { 10.0, 200.0 },   /* t=10s: subir para 200 m  */
    { 20.0,  50.0 },   /* t=20s: descer para 50 m  */
};
#define SETPOINT_COUNT  (int)(sizeof(SETPOINT_SCHEDULE) / sizeof(SETPOINT_SCHEDULE[0]))

/* Janelas de falha de sensor */
#define FAULT_START     14.0        /* Falha começa em t=14s           */
#define FAULT_END       16.0        /* Falha termina em t=16s          */
#define FAULT_VALUE     999.0       /* Altitude falsa reportada [m]    */

/* ------------------------------------------------------------------ */
/* Funções auxiliares do loop principal                               */
/* ------------------------------------------------------------------ */

/**
 * get_setpoint - Retorna o setpoint ativo no instante t.
 * Percorre a tabela de eventos e retorna o último válido.
 */
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
 * update_sensor_fault - Ativa/desativa falha de sensor conforme o tempo.
 */
static void update_sensor_fault(AltitudeSensor *sensor, double t)
{
    if (t >= FAULT_START && t < FAULT_END) {
        sensor_inject_fault(sensor, 1, FAULT_VALUE);
    } else {
        sensor_inject_fault(sensor, 0, 0.0);
    }
}

/**
 * print_header - Imprime cabeçalho da tabela de saída.
 */
static void print_header(void)
{
    printf("%-8s  %-10s  %-10s  %-10s  %-10s  %-10s  %s\n",
           "Tempo(s)",
           "Alt_real(m)",
           "Alt_sensor(m)",
           "Veloc(m/s)",
           "Empuxo(N)",
           "Erro(m)",
           "Setpoint(m)");
    printf("%-8s  %-10s  %-10s  %-10s  %-10s  %-10s  %s\n",
           "--------",
           "----------",
           "-------------",
           "----------",
           "----------",
           "----------",
           "-----------");
}

/**
 * print_state - Imprime uma linha de estado da simulação.
 */
static void print_state(double t, const AircraftState *ac,
                        double sensor_alt, double setpoint)
{
    double error = setpoint - sensor_alt;
    printf("%8.2f  %10.3f  %13.3f  %10.3f  %10.2f  %10.3f  %.1f\n",
           t,
           ac->altitude,
           sensor_alt,
           ac->velocity,
           ac->thrust,
           error,
           setpoint);
}

/* ------------------------------------------------------------------ */
/* Loop principal                                                      */
/* ------------------------------------------------------------------ */

int main(void)
{
    /* --- Inicialização dos subsistemas --- */
    AircraftState aircraft;
    aircraft_init(&aircraft, INITIAL_ALT, AIRCRAFT_MASS);

    AltitudeSensor sensor;
    sensor_init(&sensor);

    PIDController pid;
    pid_init(&pid, PID_KP, PID_KI, PID_KD, THRUST_MIN, THRUST_MAX, DT);

    /* --- Cabeçalho --- */
    printf("\n=== SIMULADOR DE CONTROLE DE ALTITUDE (PID) ===\n");
    printf("Massa: %.0f kg | Empuxo: [%.0f, %.0f] N | dt: %.0f ms\n\n",
           AIRCRAFT_MASS, THRUST_MIN, THRUST_MAX, DT * 1000.0);
    print_header();

    /* --- Loop de controle --- */
    int   total_steps    = (int)(SIM_DURATION / DT);
    int   step;
    int   print_counter  = 0;
    double t             = 0.0;
    double prev_setpoint = get_setpoint(0.0); /* Rastreia setpoint anterior */

    for (step = 0; step <= total_steps; step++) {
        t = step * DT;

        /* 1. Atualiza falha de sensor (bônus: falha simulada) */
        update_sensor_fault(&sensor, t);

        /* 2. Leitura do sensor (com filtro de média móvel) */
        double alt_measured = sensor_read(&sensor, aircraft.altitude);

        /* 3. Obtém setpoint atual (bônus: troca de setpoint) */
        double setpoint = get_setpoint(t);

        /* 4. Reset do integrador na troca de setpoint
         *
         * Quando o setpoint muda bruscamente, o acumulador integral
         * carrega "memória" do regime anterior e causa convergência
         * lenta ou overshoot no novo alvo. O reset limpa essa memória,
         * deixando apenas P e D agirem no primeiro instante do novo alvo.
         *
         * Técnica comum em sistemas embarcados reais (bumpless transfer
         * é a versão mais sofisticada, mas este reset simples já resolve
         * o problema de offset residual observado anteriormente).
         */
        if (setpoint != prev_setpoint) {
            pid_reset(&pid);
            prev_setpoint = setpoint;
        }

        /* 5. Computa controle PID */
        double thrust = pid_compute(&pid, setpoint, alt_measured);

        /* 6. Atualiza física da aeronave */
        aircraft_update(&aircraft, thrust, DT);

        /* 7. Log a cada PRINT_EVERY iterações (≈ 100 ms) */
        if (print_counter == 0) {
            print_state(t, &aircraft, alt_measured, setpoint);
        }
        print_counter = (print_counter + 1) % PRINT_EVERY;
    }

    printf("\n=== Simulação concluída: %.1f s ===\n", SIM_DURATION);
    printf("Altitude final: %.3f m | Velocidade final: %.3f m/s\n",
           aircraft.altitude, aircraft.velocity);

    return EXIT_SUCCESS;
}