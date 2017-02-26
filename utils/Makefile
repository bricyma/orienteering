
include ../Makefile.inc

INCL_DIR += -I../
CFLAGS 	 = -g #-std=c++11

SRCS_UTILS = util.c argparser.c topology.c
OBJS_UTILS = $(SRCS_UTILS:.c=.o)

LDFLAGS   = #-L/usr/local/lib/ -lyaml-cpp 

$(LIB_UTILS): $(OBJS_UTILS) $(DEPEND)
	$(AR) $(ARFLAGS) $(LIB_UTILS) $(OBJS_UTILS) 
	$(RANLIB) $(LIB_UTILS)
	$(ECHO) Done making $(LIB_UTILS) library.
	$(ECHO)

%.o: %.cc $(DEPEND)
	$(CC) $(CFLAGS) $(INCL_DIR) -c $< $(LDFLAGS)

%.o: %.c $(DEPEND)
	$(CC) $(CFLAGS) $(INCL_DIR) -c $< $(LDFLAGS)


$(DEPEND): $(SRCS_UTILS)
	$(RM) -f $(DEPEND)
	$(CC) $(CFLAGS) $(INCL_DIR) -MM $^ > $(DEPEND); 

include $(DEPEND)

.PHONY: clean reallyclean
clean:
	$(RM) -f *.o *.a *.~ .*.swp

reallyclean:
	$(RM) -f *.o *.a *.~ .*.swp .depend*

