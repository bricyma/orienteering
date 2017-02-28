
include Makefile.inc

DIRS = $(ASSIGNMENT) $(VIZ_TOOL) $(UTILS)

INCL_DIR += -I. #needed. to locate current dir for compiling local dir sources

OBJLIBS = $(LIB_ASSIGNMENT) $(LIB_VIZ_TOOL) $(LIB_UTILS) 

LD_LIBS = -L$(VIZ_TOOL) -l$(VIZ_TOOL) \
	-L$(UTILS) -l$(UTILS) \
	-L$(ASSIGNMENT) -l$(ASSIGNMENT) \
	-L/usr/local/lib

OBJS = 

SRCS = main.cpp \
	dag.cpp\
	op.c\
	paths.c \
	render.c \
	tsp.c\
	PathPlanning.cpp

TEMP = $(SRCS:.c=.o)
OBJS = $(TEMP:.cpp=.o)

EXE_SIMU = simu
all: $(EXE_SIMU)

$(EXE_SIMU): $(OBJS) $(OBJLIBS)
	$(LD) -o $(EXE_SIMU) $(OBJS) $(LD_LIBS) $(LIBS_GL) #$(LIBS_ARMA) 
	@echo Done making $(EXE_SIMU).

$(LIB_ASSIGNMENT): force_look 
	@echo $(LIB_ASSIGNMENT) $(OBJS)
	cd $(ASSIGNMENT); $(MAKE) $(MFLAGS)

$(LIB_VIZ_TOOL): force_look
	cd $(VIZ_TOOL); $(MAKE) $(MFLAGS)

$(LIB_UTILS): force_look
	cd $(UTILS); $(MAKE) $(MFLAGS)


%.o: %.cpp
	$(CC) $(CFLAGS) $(INCL_DIR) -c $<

%.o: %.cc
	$(CC) $(CFLAGS) $(INCL_DIR) -c $<

%.o: %.c
	$(CC) $(CFLAGS) $(INCL_DIR) -c $<


.PHONY: clean reallyclean
clean:
	rm -f $(EXE_SIMU) *.o *.~ .*.swp
	-for d in $(DIRS); do (cd $$d; $(MAKE) clean ); done

reallyclean:
	rm -f $(EXE_SIMU) *.o *.~ .*.swp
	-for d in $(DIRS); do (cd $$d; $(MAKE) reallyclean ); done

force_look:
	$(ECHO) force look:
	true


