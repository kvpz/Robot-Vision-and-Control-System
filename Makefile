WARN	:= #-W -Wall -Wconversion -pedantic
CC	:= g++
USRLIBS	:= /usr/lib
RSHEADERS := /usr/include/librealsense2
#PROJHEADERS := /home/ieeefiu/Documents/perrito/include
PROJHEADERS := /home/kevin/Documents/perrito/include 
INCLUDE = -I$(RSHEADERS) -I$(PROJHEADERS)

ALL: main.x

main.x: task.o navigatetotask.o pathcorrectiontask.o taskmanager.o robot.o 
	@echo "building target main.x"
	$(CC) $(INCLUDE) $(WARN) -pthread ./src/main.cpp -O3 -L$(USRLIBS) -lrealsense2 -o main.x task.o navigatetotask.o pathcorrectiontask.o taskmanager.o robot.o

task.o: $(PROJHEADERS)
	@echo "building target task.o"
	$(CC) -c $(INCLUDE) $(WARN) -pthread ./src/task.cpp -O3 -L$(USRLIBS) -lrealsense2

navigatetotask.o: $(PROJHEADERS)
	@echo "building target navigatetotask.o"
	$(CC) -c $(INCLUDE) $(WARN) -pthread ./src/navigatetotask.cpp -O3 -L$(USRLIBS) -lrealsense2 

pathcorrectiontask.o: $(PROJHEADERS)
	@echo "building target pathcorrectiontask.o"
	$(CC) -c $(INCLUDE) $(WARN) -pthread ./src/pathcorrectiontask.cpp -O3 -L$(USRLIBS) -lrealsense2 

#posecorrectiontask.o: $(PROJHEADERS)
#	@echo "building target posecorrectiontask.o"
#	$(CC) -c $(INCLUDE) $(WARN) -pthread ./src/posecorrectiontask.cpp -O3 -L$(USRLIBS) -lrealsense2 

taskmanager.o: $(PROJHEADERS)
	@echo "building target taskmanager.o"
	$(CC) -c $(INCLUDE) $(WARN) -pthread ./src/taskmanager.cpp -O3 -L$(USRLIBS) -lrealsense2 

robot.o: ./src/robot.cpp $(PROJHEADERS) 
	$(CC) -c $(INCLUDE) $(WARN) -pthread -O3 -L$(USRLIBS) -lrealsense2 -c ./src/robot.cpp

clean:
	rm -rf *.o *.x



