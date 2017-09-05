CC = g++
SRCS = $(wildcard *.c)
OBJS = $(SRCS:.c=.o)
BIN = test
CFLAGS = -Wall -O2

$(BIN): $(OBJS)
	$(CC) $^ -o $@

.PHONY: clean
clean:
	$(RM) $(BIN) $(OBJS)
