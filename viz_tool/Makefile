
include ../Makefile.inc

INCL_DIR += -I../

SRCS_VIZ_TOOL = canvas.c \
	glm.c \
	gltb.c \
	glfunc.c \
	gldraw.c \
	shadow.c \
	floor.c \
	bmploader.c \
	texture.c \
	tex_walls.c 

OBJS_VIZ_TOOL = $(SRCS_VIZ_TOOL:.c=.o) 

$(LIB_VIZ_TOOL): $(OBJS_VIZ_TOOL) $(DEPEND)
	$(AR) $(ARFLAGS) $(LIB_VIZ_TOOL) $(OBJS_VIZ_TOOL) 
	$(RANLIB) $(LIB_VIZ_TOOL)
	$(ECHO) Done making $(LIB_VIZ_TOOL) library.
	$(ECHO)

%.o: %.c $(DEPEND)
	$(CC) $(CFLAGS) $(INCL_DIR) -c $< 

$(DEPEND): $(SRCS_VIZ_TOOL)
	$(RM) -f $(DEPEND)
	$(CC) $(CFLAGS) $(INCL_DIR) -MM $^ > $(DEPEND); 

include $(DEPEND)

.PHONY: clean reallyclean
clean:
	$(RM) -f *.o *.a *.~ .*.swp

reallyclean:
	$(RM) -f *.o *.a *.~ .*.swp .depend*


