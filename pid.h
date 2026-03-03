#ifndef PID_H
#define PID_H

/**
 * pid.h - Controlador PID discreto com ganhos adaptativos
 *
 * Funcionalidades:
 *   - Anti-windup por clamping condicional
 *   - Saturação de saída (limites min/max)
 *   - Ganhos adaptativos: Kp e Kd variam conforme a magnitude do erro
 *     (agressivo longe do alvo, suave perto do alvo)
 */

/* ------------------------------------------------------------------ */
/* Zona de ganho adaptativo                                            */
/* Define Kp/Ki/Kd para uma faixa de erro específica.                 */
/* ------------------------------------------------------------------ */
typedef struct {
    double error_threshold; /* Erro abaixo do qual esta zona é ativada [m] */
    double kp;
    double ki;
    double kd;
} GainZone;

typedef struct {
    /* Ganhos ativos (atualizados a cada iteração pelo adaptador) */
    double kp;
    double ki;
    double kd;

    /* Limites de saída */
    double out_min;
    double out_max;

    /* Estado interno */
    double integral;
    double prev_error;
    double prev_measurement; /* Usado para D sobre medição (evita derivative kick) */
    double dt;

    /* Ganhos adaptativos */
    const GainZone *zones;      /* Tabela de zonas (ordenada do menor erro ao maior) */
    int             zone_count; /* Número de zonas na tabela                         */
} PIDController;

/**
 * pid_init - Inicializa o controlador com ganhos fixos (sem adaptação).
 */
void pid_init(PIDController *pid, double kp, double ki, double kd,
              double out_min, double out_max, double dt);

/**
 * pid_set_adaptive_gains - Configura a tabela de zonas de ganho adaptativo.
 *
 * As zonas devem ser ordenadas do MENOR para o MAIOR error_threshold.
 * Exemplo:
 *   zones[0] = { 10.0, kp=40,  ki=20, kd=150 }  → erro < 10m
 *   zones[1] = { 40.0, kp=80,  ki=20, kd=250 }  → erro < 40m
 *   zones[2] = { 999., kp=120, ki=20, kd=350 }  → erro >= 40m
 */
void pid_set_adaptive_gains(PIDController *pid,
                            const GainZone *zones, int zone_count);

/**
 * pid_reset - Zera o estado interno (integral e erros anteriores).
 */
void pid_reset(PIDController *pid);

/**
 * pid_decay_integral - Aplica um decaimento exponencial à integral.
 *
 * Em vez de zerar abruptamente ao trocar de setpoint, reduz a integral
 * por um fator (0 < factor < 1) a cada chamada, produzindo uma transição
 * suave sem spike de empuxo.
 *
 * @factor : fração a manter por iteração (ex.: 0.5 = decai à metade)
 */
void pid_decay_integral(PIDController *pid, double factor);

/**
 * pid_compute - Calcula a saída do controlador.
 *
 * Se ganhos adaptativos estiverem configurados, os ganhos são
 * atualizados automaticamente com base no erro atual.
 *
 * O termo derivativo é calculado sobre a MEDIÇÃO (não sobre o erro),
 * evitando "derivative kick" em trocas bruscas de setpoint.
 *   D = -Kd * (measurement - prev_measurement) / dt
 */
double pid_compute(PIDController *pid, double setpoint, double measurement);

/**
 * pid_get_active_zone - Retorna o índice da zona de ganho atualmente ativa.
 * Útil para logging e debug. Retorna -1 se não houver zonas configuradas.
 */
int pid_get_active_zone(const PIDController *pid, double error);

#endif /* PID_H */