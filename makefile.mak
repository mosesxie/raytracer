OBJ = raytracer.o
CC = g++
DIA = -std=c++11
OPT = -O3
DEBUG = -g3
CFLAGS = -Wall -c $(DEBUG)
LFLAGS = -Wall $(DEBUG)

all : $(OBJ)
	$(CC) $(DIA) $(OPT) $(LFLAGS) $(OBJ) -o raytracer
	
raytracer.o : raytracer.hpp raytracer.cpp
	$(CC) $(DIA) $(OPT) $(CFLAGS) raytracer.cpp
	
clean : 
	del *.o raytracer.exe