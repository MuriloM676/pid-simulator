#ifndef AIRCRAFT_H
#define AIRCRAFT_H

/**
 * aircraft.h - Modelo físico simplificado de aeronave (voo vertical)
 *
 * Dinâmica:
 *   aceleracao  = (empuxo - peso) / massa
 *   velocidade += aceleracao * dt
 *   altitude   += velocidade * dt
 *
 * Sensor simulado com filtro de média móvel e opção de falha injetada.
 */

#define SENSOR_FILTER_SIZE 5    /* Tamanho da janela do filtro de média móvel */
#define GRAVITY            9.81 /* Aceleração gravitacional [m/s²] */

/* ------------------------------------------------------------------ */
/* Estrutura de estado da aeronave                                     */
/* ------------------------------------------------------------------ */
typedef struct {
    double altitude;        /* Altitude atual [m]          */
    double velocity;        /* Velocidade vertical [m/s]   */
    double mass;            /* Massa da aeronave [kg]       */
    double thrust;          /* Empuxo aplicado [N]          */
} AircraftState;

/* ------------------------------------------------------------------ */
/* Estrutura do sensor (com filtro e falha simulada)                   */
/* ------------------------------------------------------------------ */
typedef struct {
    double buffer[SENSOR_FILTER_SIZE];  /* Janela circular de leituras */
    int    index;                       /* Índice atual na janela       */
    int    count;                       /* Amostras válidas acumuladas  */
    int    fault_active;                /* 1 = falha injetada           */
    double fault_value;                 /* Valor fixo quando em falha   */
} AltitudeSensor;

/**
 * aircraft_init - Inicializa o estado da aeronave.
 *
 * @state       : ponteiro para o estado
 * @initial_alt : altitude inicial [m]
 * @mass_kg     : massa [kg]
 */
void aircraft_init(AircraftState *state, double initial_alt, double mass_kg);

/**
 * aircraft_update - Avança a física um passo de tempo dt.
 *
 * Empuxo e altitude são limitados:
 *   - altitude >= 0 (solo)
 *   - velocidade zerada ao tocar o solo
 */
void aircraft_update(AircraftState *state, double thrust, double dt);

/**
 * sensor_init - Inicializa o sensor de altitude.
 */
void sensor_init(AltitudeSensor *sensor);

/**
 * sensor_read - Lê altitude com filtro de média móvel.
 *
 * Se fault_active == 1, retorna fault_value diretamente.
 * Caso contrário, insere a leitura real no filtro e retorna a média.
 */
double sensor_read(AltitudeSensor *sensor, double true_altitude);

/**
 * sensor_inject_fault - Ativa/desativa falha de sensor.
 *
 * @active      : 1 para ativar falha, 0 para desativar
 * @fault_value : altitude falsa reportada (quando active=1)
 */
void sensor_inject_fault(AltitudeSensor *sensor, int active, double fault_value);

#endif /* AIRCRAFT_H */
