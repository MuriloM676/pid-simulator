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

/* ------------------------------------------------------------------ */
/* Constantes de simulação                                             */
/* ------------------------------------------------------------------ */
#define DT              0.01
#define SIM_DURATION    40.0
#define PRINT_EVERY     10          /* Log no terminal a cada 100 ms   */

#define AIRCRAFT_MASS   500.0
#define THRUST_MAX      10000.0
#define THRUST_MIN      0.0
#define INITIAL_ALT     0.0

#define PID_KP          120.0
#define PID_KI          20.0
#define PID_KD          350.0

#define SETPOINT_RATE_MAX   8.0     /* Rate limiter [m/s]              */
#define LANDING_ALT         20.0    /* Altitude de ativação pouso [m]  */
#define LANDING_VMAX        -2.0    /* Velocidade máx descida [m/s]    */

#define CSV_PATH        "/tmp/pid_sim_data.csv"
#define PNG_PATH        "/home/muril/pid_simulator/simulation_result.png"

/* ------------------------------------------------------------------ */
/* Buffer de dados para o gráfico                                      */
/* Armazena uma amostra a cada DT (10ms) → 4001 amostras em 40s      */
/* ------------------------------------------------------------------ */
#define MAX_SAMPLES     4002   /* (40.0 / 0.01) + 2 amostras   */

typedef struct {
    double t;
    double altitude;
    double sensor;
    double setpoint;
    double velocity;
    double thrust;
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

#define FAULT_START  14.0
#define FAULT_END    16.0
#define FAULT_VALUE  999.0

/* ------------------------------------------------------------------ */
/* Funções auxiliares                                                  */
/* ------------------------------------------------------------------ */

static double get_setpoint(double t)
{
    double sp = SETPOINT_SCHEDULE[0].setpoint;
    int i;
    for (i = 0; i < SETPOINT_COUNT; i++)
        if (t >= SETPOINT_SCHEDULE[i].time)
            sp = SETPOINT_SCHEDULE[i].setpoint;
    return sp;
}

static double apply_rate_limiter(double current, double target, double dt)
{
    double delta    = target - current;
    double max_step = SETPOINT_RATE_MAX * dt;
    if (delta >  max_step) return current + max_step;
    if (delta < -max_step) return current - max_step;
    return target;
}

static double apply_landing_mode(double sp, double altitude, double velocity)
{
    if (altitude < LANDING_ALT && velocity < LANDING_VMAX) {
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
    printf("%-8s  %-10s  %-10s  %-10s  %-10s  %-10s  %-12s  %s\n",
           "Tempo(s)", "Alt_real(m)", "Alt_sensor(m)", "Veloc(m/s)",
           "Empuxo(N)", "Erro(m)", "SP_ramp(m)", "Modo");
    printf("%-8s  %-10s  %-10s  %-10s  %-10s  %-10s  %-12s  %s\n",
           "--------", "----------", "-------------", "----------",
           "----------", "----------", "------------", "----");
}

static void print_state(double t, const AircraftState *ac,
                        double sensor_alt, double sp_ramp)
{
    double error      = sp_ramp - sensor_alt;
    const char *mode  = "NORMAL";
    if (ac->altitude < LANDING_ALT && ac->velocity < LANDING_VMAX)
        mode = "POUSO";
    else if (t >= FAULT_START && t < FAULT_END)
        mode = "FALHA_SNS";

    printf("%8.2f  %10.3f  %13.3f  %10.3f  %10.2f  %10.3f  %12.3f  %s\n",
           t, ac->altitude, sensor_alt, ac->velocity,
           ac->thrust, error, sp_ramp, mode);
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
        double sensor_plot = (g_samples[i].sensor > 500.0)
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
    pclose(gp);
    printf("\nImagem salva em: %s\n", PNG_PATH);

    /* ── 2. Abre janela interativa ── */
    gp = popen("gnuplot -persistent", "w");
    if (!gp) return;
    fprintf(gp, "set terminal qt size 1100,750 title 'PID - Simulador de Altitude'\n");
    send_plot_commands(gp);
    pclose(gp);
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
    double target_sp      = get_setpoint(0.0);
    double ramp_sp        = get_setpoint(0.0);
    double prev_target_sp = target_sp;

    for (step = 0; step <= total_steps; step++) {
        t = step * DT;

        /* 1. Falha de sensor */
        update_sensor_fault(&sensor, t);

        /* 2. Leitura do sensor */
        double alt_measured = sensor_read(&sensor, aircraft.altitude);

        /* 3. Setpoint alvo */
        target_sp = get_setpoint(t);

        /* 4. Reset do integrador na troca de setpoint */
        if (target_sp != prev_target_sp) {
            pid_reset(&pid);
            prev_target_sp = target_sp;
        }

        /* 5. Rate limiter */
        ramp_sp = apply_rate_limiter(ramp_sp, target_sp, DT);

        /* 6. Modo pouso suave */
        double pid_sp = apply_landing_mode(ramp_sp, aircraft.altitude,
                                           aircraft.velocity);

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
            g_sample_count++;
        }

        /* 10. Log no terminal a cada 100ms */
        if (print_counter == 0)
            print_state(t, &aircraft, alt_measured, pid_sp);
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