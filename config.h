/**
 * config.h - Parâmetros de configuração do simulador de altitude
 *
 * Centraliza todas as constantes ajustáveis em um único lugar,
 * facilitando experimentos sem necessidade de alterar a lógica.
 */

#ifndef CONFIG_H
#define CONFIG_H

/* ------------------------------------------------------------------ */
/* Simulação                                                           */
/* ------------------------------------------------------------------ */
#define DT              0.01    /* Passo de integração [s]             */
#define SIM_DURATION    40.0    /* Duração total da simulação [s]      */
#define PRINT_EVERY     10      /* Log no terminal a cada N iterações  */
                                /* (N * DT = 100 ms)                   */

/* ------------------------------------------------------------------ */
/* Aeronave                                                            */
/* ------------------------------------------------------------------ */
#define AIRCRAFT_MASS   500.0   /* Massa [kg]                          */
#define THRUST_MAX      10000.0 /* Empuxo máximo [N]                   */
#define THRUST_MIN      0.0     /* Empuxo mínimo [N] (sem negativo)    */
#define INITIAL_ALT     0.0     /* Altitude inicial [m]                */

/* ------------------------------------------------------------------ */
/* Ganhos PID                                                          */
/* ------------------------------------------------------------------ */
#define PID_KI          20.0   /* Ki fixo em todas as zonas            */

/* Zonas de ganho adaptativo: { error_threshold, kp, ki, kd }
 * Ordenadas do MENOR para o MAIOR threshold.                         */
#define PID_ZONE_SOFT   {  15.0,  60.0, PID_KI, 600.0 }  /* erro < 15m  */
#define PID_ZONE_MID    {  50.0,  90.0, PID_KI, 450.0 }  /* erro < 50m  */
#define PID_ZONE_AGG    { 999.0, 120.0, PID_KI, 350.0 }  /* erro >= 50m */

/* ------------------------------------------------------------------ */
/* Setpoint                                                            */
/* ------------------------------------------------------------------ */
#define SETPOINT_RATE_MAX   8.0   /* Rate limiter do setpoint [m/s]    */

/* ------------------------------------------------------------------ */
/* Modo de pouso suave                                                 */
/* ------------------------------------------------------------------ */
#define LANDING_ALT         20.0  /* Altitude de ativação pouso [m]    */
#define LANDING_VMAX        (-2.0)/* Velocidade máx de descida [m/s]   */

/* ------------------------------------------------------------------ */
/* Falha de sensor simulada                                            */
/* ------------------------------------------------------------------ */
#define FAULT_START  14.0   /* Início da janela de falha [s]           */
#define FAULT_END    16.0   /* Fim da janela de falha [s]              */
#define FAULT_VALUE  999.0  /* Valor fixo injetado durante a falha [m] */

/* ------------------------------------------------------------------ */
/* Fade-out do integrador na troca de setpoint                        */
/* ------------------------------------------------------------------ */
#define INTEGRAL_DECAY_STEPS   20    /* Duração do fade: N * DT = 200ms */
#define INTEGRAL_DECAY_FACTOR  0.75  /* Fração mantida por iteração      */

/* ------------------------------------------------------------------ */
/* Arquivos de saída                                                   */
/* ------------------------------------------------------------------ */
#define CSV_PATH  "/tmp/pid_sim_data.csv"
#define PNG_PATH  "simulation_result.png"

/* ------------------------------------------------------------------ */
/* Buffer de amostras                                                  */
/* ------------------------------------------------------------------ */
/* (SIM_DURATION / DT) + 2 amostras de margem = (40.0/0.01)+2 = 4002 */
#define MAX_SAMPLES  4002

#endif /* CONFIG_H */

