#ifndef ROBOT_HPP
#define ROBOT_HPP
using namespace std;

enum RobotState {
    IDLE,
    MOVING_FORWARD,
    MOVE_LEFT,
    MOVE_RIGHT,
    ROTATE_LEFT,
    ROTATE_RIGHT,
    STOPPED
};

class Robot {
public:
    void setState(RobotState newState) {
        state = newState;
    }
    
    void run() {
        while (true) {
            switch (state) {
                case IDLE:
                    // Do nothing until commanded to move
                    break;
                case MOVING_FORWARD:
                    // Move forward until commanded to stop
                    cout << "Moving forward..." << endl;
                    break;
                case MOVE_LEFT:
                    // Turn left until commanded to stop
                    cout << "Turning left..." << endl;
                    break;
                case MOVE_RIGHT:
                    // Turn right until commanded to stop
                    cout << "Turning right..." << endl;
                    break;
                case STOPPED:
                    // Stop and wait for new command
                    cout << "Stopped." << endl;
                    break;
            }
        }
    }
    
private:
    RobotState state = IDLE;
};

#endif