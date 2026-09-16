CC		= gcc
CPP	= g++
LD		= g++
CP		= cp
CFLAGS    = -std=c99 -ggdb -I../include -I../libopeninv/include
CPPFLAGS    = -ggdb -I../include -I../libopeninv/include
LDFLAGS     = -g
BINARY		= test_vcu
OBJS		= test_main.o my_string.o params.o throttle.o test_throttle.o MGgen2Frames.o test_mggen2.o canhardware.o MGgen2Charger.o test_mggen2charger.o
VPATH = ../src ../libopeninv/src

all: $(BINARY)

$(BINARY): $(OBJS)
	$(LD) $(LDFLAGS) -o $(BINARY) $(OBJS)

# Linux-only bench tool: runs MGgen2Charger against a real charger over
# SocketCAN. Not built by "all"; run "make mg_bench" when you need it.
BENCH		= mg_bench
BENCH_OBJS	= mg_bench.o my_string.o params.o MGgen2Frames.o canhardware.o MGgen2Charger.o

$(BENCH): $(BENCH_OBJS)
	$(LD) $(LDFLAGS) -o $(BENCH) $(BENCH_OBJS)

# Same idea for the BMW i3 LIM: run i3LIMClass against a real LIM.
LIMBENCH		= lim_bench
LIMBENCH_OBJS	= lim_bench.o my_string.o params.o i3LIM.o canhardware.o

$(LIMBENCH): $(LIMBENCH_OBJS)
	$(LD) $(LDFLAGS) -o $(LIMBENCH) $(LIMBENCH_OBJS)

%.o: ../%.cpp
	$(CPP) $(CPPFLAGS) -o $@ -c $<

%.o: ../%.c
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -f $(OBJS) $(BINARY) $(BENCH_OBJS) $(BENCH) $(LIMBENCH_OBJS) $(LIMBENCH)
