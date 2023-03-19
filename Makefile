WARN	:= #-W -Wall -Wconversion -pedantic
CC	:= g++
USRLIBS	:= /usr/lib
RSHEADERS := /usr/include/librealsense2
#PROJHEADERS := /home/ieeefiu/Documents/perrito/include
PROJHEADERS := /home/kevin/Documents/perrito/include 
INCLUDE = -I$(RSHEADERS) -I$(PROJHEADERS)

ALL: main.x

main.x: robot.o task.o travelTask.o taskmanager.o #./src/main.cpp $(PROJHEADERS) #robot.o
	@echo "building target main.x"
	$(CC) $(INCLUDE) $(WARN) -pthread ./src/main.cpp -O3 -L$(USRLIBS) -lrealsense2 -o main.x robot.o task.o travelTask.o taskmanager.o 

robot.o: ./src/robot.cpp $(PROJHEADERS) 
	$(CC) -c $(INCLUDE) $(WARN) -pthread -O3 -L$(USRLIBS) -lrealsense2 -c ./src/robot.cpp

# this compiles
task.o: $(PROJHEADERS)
	@echo "building target task.o"
	$(CC) -c $(INCLUDE) $(WARN) -pthread ./src/task.cpp -O3 -L$(USRLIBS) -lrealsense2


travelTask.o: $(PROJHEADERS)
	@echo "building target travelTask.o"
	$(CC) -c $(INCLUDE) $(WARN) -pthread ./src/travelTask.cpp -O3 -L$(USRLIBS) -lrealsense2 


taskmanager.o: $(PROJHEADERS)
	@echo "building target taskmanager.o"
	$(CC) -c $(INCLUDE) $(WARN) -pthread ./src/taskmanager.cpp -O3 -L$(USRLIBS) -lrealsense2 




clean:
	rm -rf *.o *.x
