#ifndef PID_H
#define PID_H

/**
 * pid.h - Controlador PID com anti-windup e saturação de saída
 *
 * Implementa um controlador PID discreto com:
 *   - Anti-windup por clamping integral
 *   - Saturação de saída (limites min/max)
 *   - Derivada aplicada sobre o erro (não sobre a saída)
 */

typedef struct {
    /* Ganhos */
    double kp;          /* Ganho proporcional */
    double ki;          /* Ganho integral     */
    double kd;          /* Ganho derivativo   */

    /* Limites de saída */
    double out_min;     /* Saída mínima (ex: empuxo mínimo) */
    double out_max;     /* Saída máxima (ex: empuxo máximo) */

    /* Estado interno */
    double integral;    /* Acumulador integral */
    double prev_error;  /* Erro na iteração anterior (para derivada) */

    /* Período de amostragem */
    double dt;          /* Intervalo de tempo [s] */
} PIDController;

/**
 * pid_init - Inicializa o controlador PID com os parâmetros fornecidos.
 */
void pid_init(PIDController *pid, double kp, double ki, double kd,
              double out_min, double out_max, double dt);

/**
 * pid_reset - Zera o estado interno (integral e erro anterior).
 */
void pid_reset(PIDController *pid);

/**
 * pid_compute - Calcula a saída do controlador dado o setpoint e a medição atual.
 *
 * Retorna o valor de controle (ex: empuxo necessário).
 * Aplica anti-windup: o integral só acumula quando a saída não está saturada.
 */
double pid_compute(PIDController *pid, double setpoint, double measurement);

#endif /* PID_H */
