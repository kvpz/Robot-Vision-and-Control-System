# AMIRA Robot Vision and Control System

## 1. Task Scheduler
A task in the task scheduler is designed to perform the following:
1. Read the current state of the state machine.
2. Determine the next state of the state machine based on the current state and any inputs.
3. Perform any necessary actions associated with the current state, such as reading sensors, controlling actuators, or communicating with other systems.
4. Update any internal state variables used by the state machine or other tasks.
5. Request any additional tasks to be executed based on the current state.

### 1.1 Tasks
The task scheduler supports the following tasks:
* Endpoint navigation (*TRAVEL*)
* Path correction (*CORRECTION*)
* Payload deployment (*DROPCHIP*)
* Target search (*SEARCH*)

#### 1.1.1 Waypoint Navigation Task

#### 1.1.2 Path Correction Task
This task makes the robot correct its path towards a destination given as a (x.y) point on a cartesian coordinate system. The strategy to correct the robot's path to the destination is to calculate its orientation relative to the destination. The robot's orientation relative to the destination is the slope angle between the robot's current location and destination. This is calculated using the arctangent of the `(rise/run)`.

#### 1.1.3 Payload Deployment Task
This task makes the robot drop its payload based on an autonomous decision.
It uses the information gathered from an RGB camera to decisively drop the correct payload. 
The robot will carry red and green chips; each one must be placed over a red or green rectangular area, respectively. 
This task may become a subtask, or action, that will be tied to a waypoint. In other words, when the robot arrives at an endpoint, it can perform an action corresponding to that location.

#### 1.1.4 Target Search Task
This task is responsible for moving the robot until it has identified an object of interest.
Objects of interest are: yellow ducks and green, red, or white cylinders. 
