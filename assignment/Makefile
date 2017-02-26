
include ../Makefile.inc

INCL_DIR += -I../

DIRS = . plot

PLOTDIR = plot
PLOTOBJS = $(PLOTDIR)/Cgnuplot.o
HGRN_SCRS = Hungarian.cpp Assignment.cpp BipartiteGraph.cpp PlotGraph.c
HGRN_OBJS = Hungarian.o Assignment.o BipartiteGraph.o PlotGraph.o

OBJS = $(HGRN_OBJS) $(PLOTOBJS) main.o 

EXE = hungarian


$(LIB_ASSIGNMENT): $(OBJS) $(DEPEND)
	$(AR) $(ARFLAGS) $(LIB_ASSIGNMENT) $(OBJS)
	@echo Done making $(LIB_ASSIGNMENT) library.
	@echo
#lib  should appear before EXE and all

all: $(EXE)
$(EXE): $(OBJS) $(DEPEND)
	$(CXX) $(CFLAGS) -o $(EXE) $(OBJS)
	@echo Done.

%.o: %.cpp %.h $(DEPEND)
	$(CXX) $(CFLAGS) -c $<

$(PLOTDIR)/Cgnuplot.o:
#	@for $(i) in $(DIRS); do \
#        echo "make all in $(i)..."; 
#	(cd $(i); $(MAKE) $(MFLAGS)); done
	(cd $(PLOTDIR); $(MAKE) $(MFLAGS))
	@echo Done making Cgnuplot.

 
main.o: main.cpp Matrix.h CmdParser.h $(DEPEND) 
	$(CXX) $(CFLAGS) -c $<


$(DEPEND): $(HGRN_SRCS)
	$(RM) -f $(DEPEND)
	$(CC) $(CFLAGS) $(INCL_DIR) -MM $^ > $(DEPEND); 

include $(DEPEND)


clean:
	$(RM) -f $(EXE) *.o *.a *~ *.swp
	(cd $(PLOTDIR); $(MAKE) clean )

reallyclean:
	$(RM) -f *.o *.a *.~ .*.swp .depend*
	(cd $(PLOTDIR); $(MAKE) clean )



