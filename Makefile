INCLUDE_DIRS = 
LIB_DIRS = 
CC=gcc

CDEFS=
CFLAGS= -O0 -g $(INCLUDE_DIRS) $(CDEFS)
LIBS= 

HFILES= 
CFILES= application.c

SRCS= ${HFILES} ${CFILES}
OBJS= ${CFILES:.c=.o}

all:	application

clean:
	-rm -f *.o *.d
	-rm -f application

application: application.o
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $@.o -lpthread -lwiringPi -lrt

depend:

.c.o:
	$(CC) $(CFLAGS) -c $<
