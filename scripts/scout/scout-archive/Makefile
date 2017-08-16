CFLAGS		= 
OBJS            = main.o comm.o scout.o Ndirect.o escape.o seek.o video.o
SRCS            = main.c comm.c scout.c Ndirect.c escape.c seek.c video.c

compile : 	$(OBJS)
		cc -o reactived.scout $(OBJS) -lm

vtest:		video.o vtest.o
		cc -o vtest video.o vtest.o
