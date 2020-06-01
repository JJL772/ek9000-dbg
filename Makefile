#
# Makefile for EK9000 simulator
#

CC ?= gcc
CXX ?= g++

BINDIR = bin/
OUTNAME = eksim
INCLUDES += -I/usr/local/include/ 
DEFINES += -fsanitize=address -fsanitize=undefined -D_DEBUG=1
LIBS += -lreadline -lmodbus -lc -lrt -ldl -lpthread

SRCS += $(wildcard src/*.c)

all:
	mkdir -p bin 
	$(CC) -g -o $(BINDIR)/$(OUTNAME) $(INCLUDES) $(DEFINES) $(SRCS) $(LIBS)

clean:
	rm -rf $(BINDIR)

