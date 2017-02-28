
CC	= g++  
CFLAGS 	= -g -O3 -Wall -std=c++11 #-fPIC 
LD	= g++
LDFLAGS	= 
AR	= ar
ARFLAGS	= rs
RANLIB	= ranlib
RM	= rm
ECHO	= @echo

SHELL	= /bin/sh

INCL_DIR = -I/usr/local/lib  -I/usr/include/eigen3
DEPEND   = .depend    #each sub-dir will build a .depend dir to track dependacy

LIBS_GL   =  -lGLU -lglut -lGL -lXext -lpthread
LIBS_ARMA = -larmadillo -llapack -lblas #-llapack_atlas


# naming packages/directories (libs will use the names as well)

VIZ_TOOL    = viz_tool
ASSIGNMENT  = assignment
UTILS	    = utils

LIB_ASSIGNMENT  = lib$(ASSIGNMENT).a
LIB_VIZ_TOOL    = lib$(VIZ_TOOL).a
LIB_UTILS       = lib$(UTILS).a


#uncomment below to silent output
#.SILENT :

#basics: http://owen.sj.ca.us/~rk/howto/slides/make/slides/makerecurs.html
#depend: http://stackoverflow.com/questions/2394609/makefile-header-dependencies

