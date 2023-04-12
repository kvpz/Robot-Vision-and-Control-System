WARN	:= #-pedantic #-W -Wall -Wconversion -pedantic
CC	:= g++ -std=c++2a #g++ -std=c++17
USRLIBS	:= /usr/lib 
RSHEADERS := /usr/include/librealsense2
INCLUDE = -I$(RSHEADERS) -I$(PROJHEADERS) -I/usr/include/jsoncpp

# IEEE Intel NUC specifics
SRC = /home/ieeefiu/Documents/perrito/src
PROJHEADERS := /home/ieeefiu/Documents/perrito/include

# Kevin computer specifics
#SRC = /home/kevin/Documents/perrito/src
#PROJHEADERS := /home/kevin/Documents/perrito/include

ALL: main.x

main.x: $(SRC)/main.cpp robot.o map.o task.o navigator.o attractioncolortask.o dropchiptask.o navigatetotask.o pathcorrectiontask.o posecorrectiontask.o taskmanager.o pickupobjecttask.o objectmappingtask.o followobjecttask.o controlmandiblestask.o visiondata.o controlwingstask.o
	@echo "building target main.x"
	$(CC) $(INCLUDE) $(WARN) -pthread ./src/main.cpp -O3 -L$(USRLIBS) -lboost_timer -lboost_system -lrealsense2 -o main.x robot.o map.o task.o navigator.o visiondata.o dropchiptask.o navigatetotask.o pathcorrectiontask.o attractioncolortask.o posecorrectiontask.o pickupobjecttask.o objectmappingtask.o followobjecttask.o controlmandiblestask.o controlwingstask.o taskmanager.o -lrt -ljsoncpp

map.o: $(SRC)/map.cpp $(PROJHEADERS)
	@echo "building target map.o"
	$(CC) -c $(INCLUDE) $(WARN) -pthread ./src/map.cpp -O3 -L$(USRLIBS) -lrealsense2 

task.o: $(SRC)/task.cpp $(PROJHEADERS)
	@echo "building target task.o"
	$(CC) -c $(INCLUDE) $(WARN) -pthread ./src/task.cpp -O3 -L$(USRLIBS) -lrealsense2

navigatetotask.o: $(SRC)/navigatetotask.cpp $(PROJHEADERS)
	@echo "building target navigatetotask.o"
	$(CC) -c $(INCLUDE) $(WARN) -pthread ./src/navigatetotask.cpp -O3 -L$(USRLIBS) -lrealsense2 

pathcorrectiontask.o: $(SRC)/pathcorrectiontask.cpp $(PROJHEADERS)
	@echo "building target pathcorrectiontask.o"
	$(CC) -c $(INCLUDE) $(WARN) -pthread ./src/pathcorrectiontask.cpp -O3 -L$(USRLIBS) -lrealsense2 

posecorrectiontask.o: $(SRC)/posecorrectiontask.cpp $(PROJHEADERS)
	@echo "building target posecorrectiontask.o"
	$(CC) -c $(INCLUDE) $(WARN) -pthread ./src/posecorrectiontask.cpp -O3 -L$(USRLIBS) -lrealsense2 

taskmanager.o: $(SRC)/taskmanager.cpp $(PROJHEADERS)
	@echo "building target taskmanager.o"
	$(CC) -c $(INCLUDE) $(WARN) -pthread ./src/taskmanager.cpp -O3 -L$(USRLIBS) -lrealsense2

robot.o: $(SRC)/robot.cpp $(PROJHEADERS) 
	$(CC) -c $(INCLUDE) $(WARN) -pthread ./src/robot.cpp -O3 -L$(USRLIBS) -lrealsense2 

navigator.o: $(SRC)/navigator.cpp $(PROJHEADERS) 
	$(CC) -c $(INCLUDE) $(WARN) -pthread ./src/navigator.cpp -O3 -L$(USRLIBS) -lrealsense2

dropchiptask.o:	$(SRC)/dropchiptask.cpp $(PROJHEADERS)
	$(CC) -c $(INCLUDE) $(WARN) -pthread ./src/dropchiptask.cpp -O3 -L$(USRLIBS) -lrealsense2

attractioncolortask.o: $(SRC)/attractioncolortask.cpp $(PROJHEADERS)
	$(CC) -c $(INCLUDE) $(WARN) -pthread ./src/attractioncolortask.cpp -O3 -L$(USRLIBS) -lrealsense2 -lrt

pickupobjecttask.o: $(SRC)/pickupobjecttask.cpp $(PROJHEADERS)
	$(CC) -c $(INCLUDE) $(WARN) -pthread ./src/pickupobjecttask.cpp -O3 -L$(USRLIBS) -lrealsense2 -lrt

objectmappingtask.o: $(SRC)/objectmappingtask.cpp $(PROJHEADERS)
	$(CC) -c $(INCLUDE) $(WARN) -pthread ./src/objectmappingtask.cpp -O3 -L$(USRLIBS) -ljsoncpp -lrealsense2 -lrt

followobjecttask.o: $(SRC)/followobjecttask.cpp $(PROJHEADERS)
	$(CC) -c $(INCLUDE) $(WARN) -pthread ./src/followobjecttask.cpp -O3 -L$(USRLIBS) -lrealsense2 -lrt

controlmandiblestask.o: $(SRC)/controlmandiblestask.cpp $(PROJHEADERS)
	$(CC) -c $(INCLUDE) $(WARN) -pthread ./src/controlmandiblestask.cpp -O3 -L$(USRLIBS) -lrealsense2 -lrt

controlwingstask.o: $(SRC)/controlwingstask.cpp $(PROJHEADERS)
	$(CC) -c $(INCLUDE) $(WARN) -pthread ./src/controlwingstask.cpp -O3 -L$(USRLIBS) -lrealsense2 

visiondata.o: $(SRC)/visiondata.cpp $(PROJHEADERS)
	$(CC) -c $(INCLUDE) $(WARN) -pthread ./src/visiondata.cpp -O3 -L$(USRLIBS) -lrealsense2 -lrt -ljsoncpp



clean:
	rm -rf *.o *.x



