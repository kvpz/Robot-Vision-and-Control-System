WARN	:= -W -Wall -Wconversion -pedantic
CC	:= g++
USRLIBS	:= /usr/lib
RSHEADERS := /usr/include/librealsense2
PROJHEADERS := /home/ieeefiu/Documents/perrito/include
#PROJHEADERS := /home/kevin/Documents/perrito/include 
INCLUDE = -I$(RSHEADERS) -I$(PROJHEADERS)

ALL: main.x

main.x: ./src/main.cpp $(PROJHEADERS) robot.o
	@echo "building target main.x"
	$(CC) $(INCLUDE) $(WARN) -pthread ./src/main.cpp -O3 -L$(USRLIBS) -lrealsense2 -o main.x robot.o

robot.o: ./src/robot.cpp $(PROJHEADERS) 
	$(CC) $(INCLUDE) $(WARN) -pthread -O3 -L$(USRLIBS) -lrealsense2 -c ./src/robot.cpp

clean:
	rm -rf *.o *.x
