#include <iostream>
#include <cmath>
#include <math.h>
#include <ctime>
#include "robot.hh"

using namespace gazebo;
using std::cout;
using std::endl;

typedef enum { 
    LEFT, 
    RIGHT, 
    FORWARD, 
    BACKWARD
} MOVE;

double simTime = 0;

// Pos_t range: (-PI, +PI)
// -/+ PI: SOUTH
// +: WEST
// -: EAST
#define NORTH 0
#define SOUTH M_PI
#define WEST M_PI / 2
#define EAST -M_PI / 2

bool is_following = false;
bool is_turning = false;
bool is_opening = false;
int walls_seen = 0;

// computes angle difference (positive = counter-clockwise, negative = clockwise)
// Credits: Trey
float angle_diff(float from, float to) {
    float from_adj = from < 0 ? (2 * M_PI) + from : from;
    float to_adj = to < 0 ? (2 * M_PI) + to : to;
    float raw_delta = fmodf(to_adj - from_adj, 2 * M_PI);

    if (raw_delta < -M_PI) {
        return (2 * M_PI) + raw_delta;
    }

    if (raw_delta > M_PI) {
        return (-2 * M_PI) + raw_delta;
    }

    return raw_delta;
}

// performs desired rotation on robot 
void turn_robot(Robot* robot, const MOVE type) {
    const double buffer = 0.1;
    const double leftAngleDiff = angle_diff(robot->range, WEST);
    const double rightAngleDiff = abs(angle_diff(robot->range, EAST));

    is_turning = true;

    switch(type) {
        case LEFT:
            if (leftAngleDiff > 0.01) {
                cout << leftAngleDiff << endl;
                robot->set_vel(-4.0, 4.0);

                if (leftAngleDiff > 0.005) {
                    robot->set_vel(-1.0, 1.0);
                }
            }
            break;
        case RIGHT:
            if (rightAngleDiff > 0.01) {
                cout << rightAngleDiff << endl;
                robot->set_vel(4.0, -4.0);

                if (rightAngleDiff > 0.005) {
                    robot->set_vel(1.0, -1.0);
                }
            }
            break;
        case FORWARD:
            robot->set_vel(3.0, 3.0);
            is_turning = false;
            break;
    }

    is_following = true;
}


void wall_follow(Robot* robot) {
    gazebo::common::Time currTime = gazebo::common::Time::GetWallTime();
    const double buffer = 0.1;
    const double leftAngleDiff = angle_diff(robot->range, WEST);
    const double rightAngleDiff = angle_diff(robot->range, EAST);

    // if (leftAngleDiff < buffer || rightAngleDiff < buffer || robot->range < 1.7) {
    //     robot->set_vel(5.0, 5.0);
    // }

    // spot opening
    cout << "Range: " << robot->range << endl;
    if(robot->range > 4.0) {
        // move faster if facing north or south and enter opening
        if (abs(angle_diff(robot->range, NORTH)) < buffer) {
            cout << "Going straight-ish" << endl;

            turn_robot(robot, FORWARD);
            
            // wait before turning while robot moves forward
            if (simTime == 0) {
                simTime = currTime.Double();
            }

            cout << "TIME:" << simTime - currTime.Double() << endl;
            if (currTime.Double() - simTime > 2.5) {
                is_opening = true;
            }

            // decide to turn left or right
            if (is_opening) {
                cout << "open" << endl;
                if (leftAngleDiff > 0) {
                    turn_robot(robot, LEFT);
                } else if (rightAngleDiff < 0) {
                    turn_robot(robot, RIGHT);
                }
            }
        }
    } else {
        turn_robot(robot, FORWARD);
    }
}

void callback(Robot* robot) {
    const double leftAngleDiff = angle_diff(robot->range, 1.3);
    const double rightAngleDiff = abs(angle_diff(robot->range, -1.3));

    const double buffer = 0.1;

    cout << "Robot's angle: " << robot->pos_t << endl;

    // if encounters a wall
    if (robot->range < 0.7 && robot->range != 0) {
        // check if robot is facing west or east
        cout << "Wall encountered! Turning..." << endl;
        if (leftAngleDiff > rightAngleDiff) {
            turn_robot(robot, LEFT);
        } else { 
            turn_robot(robot, RIGHT);
        }
    } else {
        // check if there is an opening
        if (is_opening && abs(angle_diff(robot->range, EAST)) < buffer) {
            turn_robot(robot, FORWARD);
            cout << "Opening!" << endl;
            return;
        }

        if (is_following) {
            cout << "Following wall!" << endl;
            wall_follow(robot);
            return;
        }

        turn_robot(robot, FORWARD);
    }
}

int main(int argc, char* argv[]) {
    cout << "making robot" << endl;
    cout << "Start!" << endl;
    Robot robot(argc, argv, callback);
    robot.do_stuff();

    return 0;
}
