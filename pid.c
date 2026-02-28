/**
 * pid.c - Implementação do controlador PID discreto
 *
 * Algoritmo:
 *   erro       = setpoint - medicao
 *   P          = kp * erro
 *   I          = integral + ki * erro * dt   (com anti-windup)
 *   D          = kd * (erro - erro_anterior) / dt
 *   saida      = clamp(P + I + D, out_min, out_max)
 *
 * Anti-windup (clamping):
 *   Se a saida saturar, o integral nao e atualizado nessa direcao.
 *   Isso evita que o integrador "estoure" quando o atuador esta limitado.
 */

#include "pid.h"

/* Utilitário: restringe valor entre [lo, hi] */
static double clamp(double value, double lo, double hi)
{
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

void pid_init(PIDController *pid, double kp, double ki, double kd,
              double out_min, double out_max, double dt)
{
    pid->kp         = kp;
    pid->ki         = ki;
    pid->kd         = kd;
    pid->out_min    = out_min;
    pid->out_max    = out_max;
    pid->dt         = dt;

    pid->integral   = 0.0;
    pid->prev_error = 0.0;
}

void pid_reset(PIDController *pid)
{
    pid->integral   = 0.0;
    pid->prev_error = 0.0;
}

double pid_compute(PIDController *pid, double setpoint, double measurement)
{
    /* 1. Calcula o erro atual */
    double error = setpoint - measurement;

    /* 2. Termo proporcional */
    double p_term = pid->kp * error;

    /* 3. Termo integral com anti-windup (clamping condicional) */
    double integral_candidate = pid->integral + pid->ki * error * pid->dt;

    /* 4. Termo derivativo (sobre o erro) */
    double d_term = pid->kd * (error - pid->prev_error) / pid->dt;

    /* 5. Saída não saturada */
    double output_raw = p_term + integral_candidate + d_term;

    /* 6. Saturação de saída */
    double output = clamp(output_raw, pid->out_min, pid->out_max);

    /* 7. Anti-windup: só atualiza integral se saída NÃO está saturada
     *    OU se a integral está "ajudando" a sair da saturação */
    int saturated_high = (output_raw >= pid->out_max);
    int saturated_low  = (output_raw <= pid->out_min);
    int wind_up_pos    = (error > 0.0) && saturated_high;
    int wind_up_neg    = (error < 0.0) && saturated_low;

    if (!wind_up_pos && !wind_up_neg) {
        pid->integral = integral_candidate;
    }

    /* 8. Salva erro para próxima iteração */
    pid->prev_error = error;

    return output;
}
