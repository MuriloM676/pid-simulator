/**
 * main.c - Loop principal do simulador de controle de altitude
 *
 * Pipeline: sensor → rate limiter → pouso suave → PID → física → log → gráfico
 *
 * Ao final da simulação, os dados são exportados para CSV e o gnuplot
 * é chamado via popen() para gerar o gráfico automaticamente.
 *
 * Gráfico gerado: 3 painéis
 *   - Painel 1: Altitude real, setpoint rampado, sensor (com falha visível)
 *   - Painel 2: Velocidade vertical
 *   - Painel 3: Empuxo
 */

#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pid.h"
#include "aircraft.h"
#include "config.h"

typedef struct {
    double t;
    double altitude;
    double sensor;
    double setpoint;
    double velocity;
    double thrust;
    int    fault;   /* 1 se leitura era de falha de sensor */
} Sample;

static Sample g_samples[MAX_SAMPLES];
static int    g_sample_count = 0;

/* ------------------------------------------------------------------ */
/* Tabela de setpoints                                                 */
/* ------------------------------------------------------------------ */
typedef struct { double time; double setpoint; } SetpointEvent;

static const SetpointEvent SETPOINT_SCHEDULE[] = {
    {  0.0, 100.0 },
    { 10.0, 200.0 },
    { 20.0,  50.0 },
};
#define SETPOINT_COUNT (int)(sizeof(SETPOINT_SCHEDULE)/sizeof(SETPOINT_SCHEDULE[0]))


/* ------------------------------------------------------------------ */
/* Funções auxiliares                                                  */
/* ------------------------------------------------------------------ */

static double get_setpoint(double t)
{
    int i;
    for (i = SETPOINT_COUNT - 1; i >= 0; i--)
        if (t >= SETPOINT_SCHEDULE[i].time)
            return SETPOINT_SCHEDULE[i].setpoint;
    return SETPOINT_SCHEDULE[0].setpoint;
}

static double apply_rate_limiter(double current, double target, double dt)
{
    double delta    = target - current;
    double max_step = SETPOINT_RATE_MAX * dt;
    if (delta >  max_step) return current + max_step;
    if (delta < -max_step) return current - max_step;
    return target;
}

static double apply_landing_mode(double sp, double altitude, double velocity,
                                 double target_sp)
{
    /* Não ativa modo de pouso se o setpoint-alvo já é baixo (<=LANDING_ALT),
     * pois isso indicaria uma descida intencional e controlada. */
    if (target_sp > LANDING_ALT &&
        altitude < LANDING_ALT && velocity < LANDING_VMAX) {
        double excess_speed = velocity - LANDING_VMAX;
        double brake_offset = -excess_speed * 3.0;
        return altitude + brake_offset;
    }
    return sp;
}

static void update_sensor_fault(AltitudeSensor *sensor, double t)
{
    if (t >= FAULT_START && t < FAULT_END)
        sensor_inject_fault(sensor, 1, FAULT_VALUE);
    else
        sensor_inject_fault(sensor, 0, 0.0);
}

/* ------------------------------------------------------------------ */
/* Terminal: cabeçalho e linha de estado                               */
/* ------------------------------------------------------------------ */

static void print_header(void)
{
    printf("%-8s  %-10s  %-10s  %-10s  %-10s  %-10s  %-12s  %-6s  %s\n",
           "Tempo(s)", "Alt_real(m)", "Alt_sensor(m)", "Veloc(m/s)",
           "Empuxo(N)", "Erro(m)", "SP_ramp(m)", "Kp", "Modo");
    printf("%-8s  %-10s  %-10s  %-10s  %-10s  %-10s  %-12s  %-6s  %s\n",
           "--------", "----------", "-------------", "----------",
           "----------", "----------", "------------", "------", "----");
}

static void print_state(double t, const AircraftState *ac,
                        double sensor_alt, double sp_ramp,
                        double kp_active)
{
    double error      = sp_ramp - sensor_alt;
    const char *mode  = "NORMAL";
    if (ac->altitude < LANDING_ALT && ac->velocity < LANDING_VMAX)
        mode = "POUSO";
    else if (t >= FAULT_START && t < FAULT_END)
        mode = "FALHA_SNS";

    printf("%8.2f  %10.3f  %13.3f  %10.3f  %10.2f  %10.3f  %12.3f  %6.1f  %s\n",
           t, ac->altitude, sensor_alt, ac->velocity,
           ac->thrust, error, sp_ramp, kp_active, mode);
}

/* ------------------------------------------------------------------ */
/* Exportação CSV                                                      */
/* ------------------------------------------------------------------ */

static int export_csv(void)
{
    FILE *fp = fopen(CSV_PATH, "w");
    if (!fp) {
        fprintf(stderr, "Erro: não foi possível criar %s\n", CSV_PATH);
        return 0;
    }

    /* Cabeçalho */
    fprintf(fp, "t,altitude,sensor,setpoint,velocity,thrust\n");

    int i;
    for (i = 0; i < g_sample_count; i++) {
        /* Oculta leituras de falha do sensor no CSV para não distorcer o gráfico
         * (substituído pelo valor real para plotagem correta) */
        double sensor_plot = g_samples[i].fault
                             ? g_samples[i].altitude   /* falha: usa real */
                             : g_samples[i].sensor;

        fprintf(fp, "%.3f,%.4f,%.4f,%.4f,%.4f,%.2f\n",
                g_samples[i].t,
                g_samples[i].altitude,
                sensor_plot,
                g_samples[i].setpoint,
                g_samples[i].velocity,
                g_samples[i].thrust);
    }

    fclose(fp);
    return 1;
}

/* ------------------------------------------------------------------ */
/* Plotagem via gnuplot                                                */
/* ------------------------------------------------------------------ */

/* Envia os comandos de plotagem para um pipe do gnuplot.
 * Usado tanto para salvar PNG quanto para exibição interativa. */
static void send_plot_commands(FILE *gp)
{
    fprintf(gp, "set datafile separator ','\n");
    fprintf(gp, "set multiplot layout 3,1 title 'Simulador PID de Controle de Altitude' font ',13'\n");
    fprintf(gp, "set grid\n");
    fprintf(gp, "set key top right\n");
    fprintf(gp, "set style line 10 lt 2 lc rgb '#888888' lw 1\n");

    /* ── Painel 1: Altitude ── */
    fprintf(gp, "set ylabel 'Altitude (m)'\n");
    fprintf(gp, "set xlabel ''\n");
    fprintf(gp, "set title 'Altitude'\n");
    fprintf(gp, "set yrange [-5:250]\n");
    fprintf(gp, "set xrange [0:%g]\n", SIM_DURATION);
    fprintf(gp, "set object 1 rect from %g,0 to %g,250 fc rgb '#FFE0E0' fs solid 0.3 noborder\n",
            FAULT_START, FAULT_END);
    fprintf(gp,
        "plot '%s' using 1:4 with lines lt 2 lc rgb '#AAAAAA' lw 2 title 'Setpoint rampado', "
             "'' using 1:2 with lines lc rgb '#2266CC' lw 2 title 'Altitude real', "
             "'' using 1:3 with lines lc rgb '#CC6622' lw 1 dt 3 title 'Sensor'\n",
        CSV_PATH);

    /* ── Painel 2: Velocidade ── */
    fprintf(gp, "unset object 1\n");
    fprintf(gp, "set object 2 rect from %g,-50 to %g,30 fc rgb '#FFE0E0' fs solid 0.3 noborder\n",
            FAULT_START, FAULT_END);
    fprintf(gp, "set title 'Velocidade Vertical'\n");
    fprintf(gp, "set ylabel 'Velocidade (m/s)'\n");
    fprintf(gp, "set yrange [-50:30]\n");
    fprintf(gp, "set ytics 10\n");
    fprintf(gp, "set arrow 1 from 0,0 to %g,0 nohead lt 0 lc rgb '#999999'\n", SIM_DURATION);
    fprintf(gp,
        "plot '%s' using 1:5 with lines lc rgb '#228833' lw 2 title 'Velocidade'\n",
        CSV_PATH);

    /* ── Painel 3: Empuxo ── */
    fprintf(gp, "unset object 2\n");
    fprintf(gp, "set object 3 rect from %g,0 to %g,11000 fc rgb '#FFE0E0' fs solid 0.3 noborder\n",
            FAULT_START, FAULT_END);
    fprintf(gp, "set title 'Empuxo'\n");
    fprintf(gp, "set ylabel 'Empuxo (N)'\n");
    fprintf(gp, "set xlabel 'Tempo (s)'\n");
    fprintf(gp, "set yrange [-500:11000]\n");
    fprintf(gp, "set ytics 2000\n");
    fprintf(gp, "unset arrow 1\n");
    fprintf(gp, "set arrow 2 from 0,%g to %g,%g nohead lt 2 lc rgb '#CC0000' lw 1\n",
            THRUST_MAX, SIM_DURATION, THRUST_MAX);
    fprintf(gp,
        "plot '%s' using 1:6 with lines lc rgb '#CC3333' lw 2 title 'Empuxo', "
             "%g with lines lt 2 lc rgb '#CC0000' lw 1 title 'Empuxo max'\n",
        CSV_PATH, THRUST_MAX);

    fprintf(gp, "unset multiplot\n");
}

static void plot_gnuplot(void)
{
    FILE *gp;
    int   status;

    /* ── 1. Salva PNG na raiz do projeto ── */
    gp = popen("gnuplot", "w");
    if (!gp) {
        fprintf(stderr, "Erro: gnuplot não encontrado. Instale com: sudo apt install gnuplot\n");
        return;
    }
    fprintf(gp, "set terminal pngcairo size 1200,800 enhanced font 'Sans,11'\n");
    fprintf(gp, "set output '%s'\n", PNG_PATH);
    send_plot_commands(gp);
    fprintf(gp, "set output\n");   /* flush e fecha o arquivo */
    status = pclose(gp);
    if (status != 0)
        fprintf(stderr, "Aviso: gnuplot (PNG) encerrou com status %d\n", status);
    else
        printf("\nImagem salva em: %s\n", PNG_PATH);

    /* ── 2. Abre janela interativa ── */
    gp = popen("gnuplot -persistent", "w");
    if (!gp) return;
    fprintf(gp, "set terminal qt size 1100,750 title 'PID - Simulador de Altitude'\n");
    send_plot_commands(gp);
    status = pclose(gp);
    if (status != 0)
        fprintf(stderr, "Aviso: gnuplot (interativo) encerrou com status %d\n", status);
    else
        printf("Gráfico interativo aberto!\n");
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

    /* Zonas de ganho adaptativo definidas em config.h.
     * Filosofia:
     *   - Longe do alvo (erro > 50m): Kp alto + Kd moderado  -> subida rápida
     *   - Perto do alvo (erro < 15m): Kp baixo + Kd muito alto -> freia forte
     *   - Interpolação suaviza a transição -> sem saltos no empuxo           */
    static const GainZone zones[] = {
        PID_ZONE_SOFT,   /* SUAVE:     erro < 15m  */
        PID_ZONE_MID,    /* MÉDIO:     erro < 50m  */
        PID_ZONE_AGG,    /* AGRESSIVO: erro >= 50m */
    };

    pid_init(&pid, 120.0, PID_KI, 350.0, THRUST_MIN, THRUST_MAX, DT);
    pid_set_adaptive_gains(&pid, zones, 3);

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
    double target_sp      = get_setpoint(0.0);
    double ramp_sp        = get_setpoint(0.0);
    double prev_target_sp = target_sp;
    int    sp_change_steps = 0;  /* Conta iterações após troca de setpoint */


    for (step = 0; step <= total_steps; step++) {
        t = step * DT;

        /* 1. Falha de sensor */
        update_sensor_fault(&sensor, t);

        /* 2. Leitura do sensor */
        double alt_measured = sensor_read(&sensor, aircraft.altitude);

        /* 3. Setpoint alvo */
        target_sp = get_setpoint(t);

        /* 4. Fade-out suave do integrador na troca de setpoint.
         *    Evita spike de empuxo sem zerar abruptamente o acúmulo I. */
        if (target_sp != prev_target_sp) {
            sp_change_steps = INTEGRAL_DECAY_STEPS;
            prev_target_sp  = target_sp;
        }
        if (sp_change_steps > 0) {
            pid_decay_integral(&pid, INTEGRAL_DECAY_FACTOR);
            sp_change_steps--;
        }

        /* 5. Rate limiter */
        ramp_sp = apply_rate_limiter(ramp_sp, target_sp, DT);

        /* 6. Modo pouso suave */
        double pid_sp = apply_landing_mode(ramp_sp, aircraft.altitude,
                                           aircraft.velocity, target_sp);

        /* 7. PID */
        double thrust = pid_compute(&pid, pid_sp, alt_measured);

        /* 8. Física */
        aircraft_update(&aircraft, thrust, DT);

        /* 9. Armazena amostra para o gráfico (toda iteração = 10ms) */
        if (g_sample_count < MAX_SAMPLES) {
            g_samples[g_sample_count].t        = t;
            g_samples[g_sample_count].altitude = aircraft.altitude;
            g_samples[g_sample_count].sensor   = alt_measured;
            g_samples[g_sample_count].setpoint = pid_sp;
            g_samples[g_sample_count].velocity = aircraft.velocity;
            g_samples[g_sample_count].thrust   = aircraft.thrust;
            g_samples[g_sample_count].fault    = sensor.fault_active;
            g_sample_count++;
        }

        /* 10. Log no terminal a cada 100ms */
        if (print_counter == 0)
            print_state(t, &aircraft, alt_measured, pid_sp, pid.kp);
        print_counter = (print_counter + 1) % PRINT_EVERY;
    }

    printf("\n=== Simulação concluída: %.1f s ===\n", SIM_DURATION);
    printf("Altitude final: %.3f m | Velocidade final: %.3f m/s\n",
           aircraft.altitude, aircraft.velocity);

    /* 11. Exporta dados e plota */
    printf("\nExportando dados para %s...\n", CSV_PATH);
    if (export_csv())
        plot_gnuplot();

    return EXIT_SUCCESS;
}