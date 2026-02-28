/**
 * aircraft.c - Modelo físico da aeronave e sensor simulado
 */

#include "aircraft.h"
#include <string.h>  /* memset */

/* ------------------------------------------------------------------ */
/* Física da aeronave                                                  */
/* ------------------------------------------------------------------ */

void aircraft_init(AircraftState *state, double initial_alt, double mass_kg)
{
    state->altitude = initial_alt;
    state->velocity = 0.0;
    state->mass     = mass_kg;
    state->thrust   = 0.0;
}

void aircraft_update(AircraftState *state, double thrust, double dt)
{
    /* Salva empuxo aplicado no estado (para logging) */
    state->thrust = thrust;

    /* Peso da aeronave [N] */
    double weight = state->mass * GRAVITY;

    /* Força resultante e aceleração vertical */
    double net_force    = thrust - weight;
    double acceleration = net_force / state->mass;

    /* Integração de Euler: velocidade e altitude */
    state->velocity += acceleration * dt;
    state->altitude += state->velocity * dt;

    /* Restrição de solo: altitude não pode ser negativa */
    if (state->altitude < 0.0) {
        state->altitude = 0.0;
        /* Zera velocidade negativa (impacto no solo) */
        if (state->velocity < 0.0) {
            state->velocity = 0.0;
        }
    }
}

/* ------------------------------------------------------------------ */
/* Sensor de altitude com filtro e falha simulada                     */
/* ------------------------------------------------------------------ */

void sensor_init(AltitudeSensor *sensor)
{
    memset(sensor->buffer, 0, sizeof(sensor->buffer));
    sensor->index        = 0;
    sensor->count        = 0;
    sensor->fault_active = 0;
    sensor->fault_value  = 0.0;
}

double sensor_read(AltitudeSensor *sensor, double true_altitude)
{
    /* Se falha ativa, retorna valor injetado diretamente */
    if (sensor->fault_active) {
        return sensor->fault_value;
    }

    /* Insere leitura real na janela circular */
    sensor->buffer[sensor->index] = true_altitude;
    sensor->index = (sensor->index + 1) % SENSOR_FILTER_SIZE;

    if (sensor->count < SENSOR_FILTER_SIZE) {
        sensor->count++;
    }

    /* Calcula média das amostras disponíveis */
    double sum = 0.0;
    int i;
    for (i = 0; i < sensor->count; i++) {
        sum += sensor->buffer[i];
    }

    return sum / (double)sensor->count;
}

void sensor_inject_fault(AltitudeSensor *sensor, int active, double fault_value)
{
    sensor->fault_active = active;
    sensor->fault_value  = fault_value;
}
