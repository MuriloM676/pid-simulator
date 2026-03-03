/**
 * pid.c - Controlador PID discreto com ganhos adaptativos
 *
 * Algoritmo base:
 *   erro  = setpoint - medicao
 *   P     = kp * erro
 *   I     = integral + ki * erro * dt   (com anti-windup)
 *   D     = kd * (erro - erro_anterior) / dt
 *   saida = clamp(P + I + D, out_min, out_max)
 *
 * Ganhos adaptativos:
 *   A cada iteração, o erro absoluto é comparado com a tabela de zonas.
 *   A zona com menor threshold que ainda seja >= |erro| é selecionada.
 *   Os ganhos Kp/Ki/Kd são atualizados suavemente (interpolação linear)
 *   para evitar saltos bruscos no sinal de controle ao cruzar zonas.
 *
 * Anti-windup (clamping condicional):
 *   Se a saída saturar, o integral não é atualizado na direção do erro.
 */

#include "pid.h"
#include <math.h>
#include <stddef.h>  /* NULL */

/* Utilitário: restringe valor entre [lo, hi] */
static double clamp(double value, double lo, double hi)
{
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

/* Utilitário: interpolação linear entre a e b pelo fator t ∈ [0,1] */
static double lerp(double a, double b, double t)
{
    return a + (b - a) * t;
}

void pid_init(PIDController *pid, double kp, double ki, double kd,
              double out_min, double out_max, double dt)
{
    pid->kp               = kp;
    pid->ki               = ki;
    pid->kd               = kd;
    pid->out_min          = out_min;
    pid->out_max          = out_max;
    pid->dt               = dt;
    pid->integral         = 0.0;
    pid->prev_error       = 0.0;
    pid->prev_measurement = 0.0;
    pid->zones            = NULL;
    pid->zone_count       = 0;
}

void pid_set_adaptive_gains(PIDController *pid,
                            const GainZone *zones, int zone_count)
{
    pid->zones      = zones;
    pid->zone_count = zone_count;
}

void pid_reset(PIDController *pid)
{
    pid->integral         = 0.0;
    pid->prev_error       = 0.0;
    pid->prev_measurement = 0.0;
}

void pid_decay_integral(PIDController *pid, double factor)
{
    pid->integral *= factor;
}

int pid_get_active_zone(const PIDController *pid, double error)
{
    double abs_error = fabs(error);
    int i;

    if (!pid->zones || pid->zone_count == 0) return -1;

    for (i = 0; i < pid->zone_count; i++) {
        if (abs_error <= pid->zones[i].error_threshold) {
            return i;
        }
    }
    /* Erro maior que todos os thresholds: usa última zona */
    return pid->zone_count - 1;
}

/**
 * adapt_gains - Atualiza kp/ki/kd com base no erro atual.
 *
 * Quando o erro está exatamente numa zona, usa seus ganhos diretamente.
 * Quando o erro está entre duas zonas, interpola linearmente os ganhos
 * para evitar saltos bruscos no sinal de controle (bumpless gain scheduling).
 */
static void adapt_gains(PIDController *pid, double error)
{
    double abs_error = fabs(error);
    int    i;

    if (!pid->zones || pid->zone_count == 0) return;

    /* Zona 0: erro menor que o menor threshold → ganhos mais suaves */
    if (abs_error <= pid->zones[0].error_threshold) {
        pid->kp = pid->zones[0].kp;
        pid->ki = pid->zones[0].ki;
        pid->kd = pid->zones[0].kd;
        return;
    }

    /* Zonas intermediárias: interpola entre zona[i-1] e zona[i] */
    for (i = 1; i < pid->zone_count; i++) {
        if (abs_error <= pid->zones[i].error_threshold) {
            double lo  = pid->zones[i-1].error_threshold;
            double hi  = pid->zones[i].error_threshold;
            double t   = (abs_error - lo) / (hi - lo); /* 0..1 */

            pid->kp = lerp(pid->zones[i-1].kp, pid->zones[i].kp, t);
            pid->ki = lerp(pid->zones[i-1].ki, pid->zones[i].ki, t);
            pid->kd = lerp(pid->zones[i-1].kd, pid->zones[i].kd, t);
            return;
        }
    }

    /* Erro maior que todos os thresholds → ganhos mais agressivos */
    pid->kp = pid->zones[pid->zone_count - 1].kp;
    pid->ki = pid->zones[pid->zone_count - 1].ki;
    pid->kd = pid->zones[pid->zone_count - 1].kd;
}

double pid_compute(PIDController *pid, double setpoint, double measurement)
{
    /* 1. Erro atual */
    double error = setpoint - measurement;

    /* 2. Atualiza ganhos adaptativos com base no erro */
    adapt_gains(pid, error);

    /* 3. Termo proporcional */
    double p_term = pid->kp * error;

    /* 4. Candidato a integral */
    double integral_candidate = pid->integral + pid->ki * error * pid->dt;

    /* 5. Termo derivativo calculado sobre a MEDIÇÃO (não sobre o erro).
     *    Evita "derivative kick" quando o setpoint muda bruscamente.
     *    d(erro)/dt = -d(medição)/dt para setpoint constante → sinal negativo. */
    double d_term = -pid->kd * (measurement - pid->prev_measurement) / pid->dt;

    /* 6. Saída bruta e saturada */
    double output_raw = p_term + integral_candidate + d_term;
    double output     = clamp(output_raw, pid->out_min, pid->out_max);

    /* 7. Anti-windup: não acumula integral se saturado na mesma direção */
    int saturated_high = (output_raw >= pid->out_max);
    int saturated_low  = (output_raw <= pid->out_min);
    int wind_up_pos    = (error > 0.0) && saturated_high;
    int wind_up_neg    = (error < 0.0) && saturated_low;

    if (!wind_up_pos && !wind_up_neg) {
        pid->integral = integral_candidate;
    }

    /* 8. Salva estado para próxima iteração */
    pid->prev_error       = error;
    pid->prev_measurement = measurement;

    return output;
}