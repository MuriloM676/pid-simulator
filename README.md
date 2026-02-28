# Simulador de Controle de Altitude com PID

Simulador em linguagem C de um sistema de controle de altitude de aeronave utilizando um controlador PID discreto. Desenvolvido com foco em arquitetura estilo sistema embarcado: modular, determinístico e sem dependências externas.

---

## Funcionalidades

- Modelo físico de voo vertical (empuxo, peso, inércia)
- Controlador PID com **anti-windup** por clamping condicional
- **Reset do integrador** na troca de setpoint
- Filtro de **média móvel** no sensor de altitude
- **Falha de sensor simulada** em janela de tempo configurável
- **Troca de setpoint** durante a simulação
- Loop de controle determinístico com período fixo (10 ms)
- Saída formatada em tabela a cada 100 ms

---

## Arquitetura

```
pid_simulator/
├── main.c        # Loop principal, scheduler, eventos
├── pid.c / .h    # Controlador PID discreto
├── aircraft.c/.h # Modelo físico da aeronave + sensor
├── CMakeLists.txt
└── Makefile
```

Cada módulo tem responsabilidade única:

| Arquivo | Responsabilidade |
|---|---|
| `pid.c` | Cálculo P, I, D — anti-windup — saturação de saída |
| `aircraft.c` | Integração de Euler — restrição de solo — sensor com filtro e falha |
| `main.c` | Loop de controle — agenda de setpoints — logging |

---

## Como compilar e rodar

### Via Make
```bash
make        # compila
make run    # compila e executa
make clean  # remove binários
```

### Via CMake (CLion)
Abra a pasta do projeto no CLion. O `CMakeLists.txt` é detectado automaticamente.
Selecione o target `altitude_sim` e pressione **Shift+F10**.

### Compilação manual
```bash
gcc -std=c99 -Wall -O2 -o altitude_sim main.c pid.c aircraft.c && ./altitude_sim
```

---

## Parâmetros configuráveis

Todos os parâmetros estão centralizados em `main.c` como `#define`:

```c
/* Simulação */
#define DT              0.01    // Período do loop [s]
#define SIM_DURATION    40.0    // Duração total [s]

/* Aeronave */
#define AIRCRAFT_MASS   500.0   // Massa [kg]
#define THRUST_MAX      10000.0 // Empuxo máximo [N]
#define THRUST_MIN      0.0     // Empuxo mínimo [N]

/* Ganhos PID */
#define PID_KP          120.0
#define PID_KI          20.0
#define PID_KD          350.0
```

### Agenda de setpoints
Editável diretamente no array `SETPOINT_SCHEDULE` em `main.c`:
```c
{ 0.0,  100.0 },   // t=0s  → subir para 100m
{ 10.0, 200.0 },   // t=10s → subir para 200m
{ 20.0,  50.0 },   // t=20s → descer para 50m
```

---

## Saída esperada

```
=== SIMULADOR DE CONTROLE DE ALTITUDE (PID) ===
Massa: 500 kg | Empuxo: [0, 10000] N | dt: 10 ms

Tempo(s)  Alt_real(m)  Alt_sensor(m)  Veloc(m/s)  Empuxo(N)   Erro(m)     Setpoint(m)
--------  ----------  -------------  ----------  ----------  ----------  -----------
    0.00       0.001          0.000       0.102    10000.00     100.000  100.0
    0.10       0.067          0.038       1.121    10000.00      99.962  100.0
    ...
   14.00     163.205        999.000      15.341        0.00    -799.000  200.0  ← falha de sensor
    ...
   20.00     196.290        196.042       8.189        0.00    -146.042   50.0  ← troca de setpoint
```

Eventos visíveis na saída:
- `t=0–10s` → subida e estabilização em **100m** (overshoot ~14%)
- `t=10s` → troca de setpoint para **200m**, integral resetado
- `t=14–16s` → **falha de sensor** (sensor reporta 999m)
- `t=20s` → troca de setpoint para **50m**, integral resetado

---

## Conceitos demonstrados

- Controle PID discreto (Euler)
- Anti-windup por clamping condicional
- Reset de integrador em troca de setpoint (*bumpless transfer* simplificado)
- Filtro de média móvel em sensor simulado
- Injeção de falha de sensor
- Loop determinístico estilo RTOS
- Saturação de atuador

---

## Requisitos

- GCC com suporte a C99 (`-std=c99`)
- `make` (opcional)
- CMake 3.10+ (opcional, para CLion)
- Nenhuma biblioteca externa além da **standard library**