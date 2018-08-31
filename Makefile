PWD=$(shell pwd)/ 
ifeq ($(ARCH), arm)
CC=arm-none-linux-gnueabi-gcc
else
CC=gcc
endif

ifeq ($(MODE), read)
PROGRAM=Tread
CFLAGS+=-Drecv
else
PROGRAM=Tsend
CFLAGS+=-Dsend
endif

#CFLAGS= -std=c99 -pie -fPIE
CFLAGS+= -std=c99  -g

SRCS=$(wildcard *.c)
OBJS=$(patsubst %.c, %.o, $(SRCS))


all:${PROGRAM}

${PROGRAM}:${OBJS}
	$(CC) $(CFLAGS) ${LDFLAGS} ${OBJS} -o $@ 

%.o:%.c 
	$(CC) -c $(CFLAGS) $(PWD)$< -o $@ 

.PHONY : clean
clean:
	-rm *.o
