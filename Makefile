# Makefile - Simulador de Controle de Altitude PID
#
# Uso:
#   make          -> compila o simulador
#   make run      -> compila e executa
#   make clean    -> remove binarios

CC      = gcc
CFLAGS  = -std=c99 -Wall -Wextra -pedantic -O2
TARGET  = altitude_sim
SRCS    = main.c pid.c aircraft.c
OBJS    = $(SRCS:.c=.o)

.PHONY: all run clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ -lm

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)

# Dependencias de cabecalho
main.o:     main.c pid.h aircraft.h
pid.o:      pid.c pid.h
aircraft.o: aircraft.c aircraft.h