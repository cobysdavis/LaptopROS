# FlyCapture2Test_C makefile
# To compile the debug verison need to overwrite CXXFLAGS variable to include -ggdb

CC = gcc
#OUTPUTNAME = FlyCapture2Test_C${D}
#INCLUDE = -I../../include -I/usr/include/flycapture
#LIBS = -L../../lib/C -lflycapture-c${D} -L../../lib -lflycapture${D}
LIBS=-lm

#OUTDIR = ../../bin/C

OUTPUTNAME = reactived.scout_xxx${D}
#INCLUDE = -I/usr/include/flycapture
#LIBS =  -lflycapture-c${D} 

OUTDIR = ./


OBJS = main.o comm.o scout.o Ndirect.o escape.o seek.o video.o 

${OUTPUTNAME}: ${OBJS}
	${CC} -o ${OUTPUTNAME} ${LIBS} ${OBJS} -lm
ifneq (${D}, d)
	strip --strip-unneeded ${OUTPUTNAME}
endif
	# mv ${OUTPUTNAME} ${OUTDIR}

%.o: %.c
	${CC} ${CFLAGS} ${INCLUDE} -c $*.c
	
clean_obj:
	rm -f ${OBJS}

clean:
	rm -f ${OUTDIR}/${OUTPUTNAME} ${OBJS}
