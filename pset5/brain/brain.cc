 #include <iostream>
#include <cmath>
#include <math.h>

#include "robot.hh"

using std::cout;
using std::endl;

typedef enum { 
    LEFT, 
    RIGHT, 
    FORWARD, 
    BACKWARD 
} MOVE;


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
                robot->set_vel(-1.0, 1.0);
            }
            break;
        case RIGHT:
            if (rightAngleDiff > 0.01) {
                cout << rightAngleDiff << endl;
                robot->set_vel(1.0, -1.0);
            }
            break;
        case FORWARD:
            robot->set_vel(3.0, 3.0);
            is_turning = false;
            break;
    }

    is_following = true;
}
    // turn left
    // if (leftAngleDiff > 0.01) {
    //     is_turning = true;
    //     cout << leftAngleDiff << endl;
    //     robot->set_vel(-1.0, 1.0);
    //     walls_seen++
    // } else {
    //     is_turning = false;
    //     cout << "RANGE NOW: " << robot->range << endl;
    //     robot->set_vel(3.0, 3.0);
    // }

    // cout << "Left: " << leftAngleDiff << endl;
    // cout << "Right: " << rightAngleDiff << endl;

    // check if turning left or right is possible
    // while(leftAngleDiff > buffer || rightAngleDiff > buffer) {
    //     if (leftAngleDiff > buffer && leftAngleDiff < WEST) {
    //         cout << "TURNING LEFT: " << leftAngleDiff << endl;
    //         robot->set_vel(-3.0, 3.0);
    //     }
        
    //     if (rightAngleDiff > buffer && rightAngleDiff < EAST) {
    //         cout << "TURNING RIGHT" << endl;
    //         robot->set_vel(3.0, -3.0);
    //     }
    //     cout << "Repeat" << endl;
    // }

    // return;
    // cout << angle << endl;
    // if (angle == float(WEST)) {
    //     // turn left from NORTH to WEST
    //     while (robot->range < angle) {
    //         cout << "TURNING LEFT: " << robot->range << " Angle: " << angle << endl;
    //         robot->set_vel(-3.0, 3.0);
    //     }
    // } else if (angle == float(EAST)) {
    //     while (robot->range > angle) {
    //         cout << "TURNING RIGHT" << robot->range << endl;
    //         robot->set_vel(3.0, -3.0);
    //     }
    // } else {
    //     // robot->set_vel(3.0, 3.0);
    // }
// }if (leftAngleDiff > 0.01) {
    //     is_turning = true;
    //     cout << leftAngleDiff << endl;
    //     robot->set_vel(-1.0, 1.0);
    //     walls_seen++
    // } else {
    //     is_turning = false;
    //     cout << "RANGE NOW: " << robot->range << endl;
    //     robot->set_vel(3.0, 3.0);
    // }


void wall_follow(Robot* robot) {
    const double buffer = 0.1;
    const double leftAngleDiff = angle_diff(robot->range, WEST);
    const double rightAngleDiff = abs(angle_diff(robot->range, EAST));


    // if (leftAngleDiff < buffer || rightAngleDiff < buffer || robot->range < 1.7) {
    //     robot->set_vel(5.0, 5.0);
    // }

    // move faster if facing north or south and enter opening
    // cout << angle_diff(robot->range, NORTH) << endl;
    if (abs(angle_diff(robot->range, NORTH)) < buffer && robot->range < 4.0) {
        cout << "Going straight-ish" << endl;
        robot->set_vel(5.0, 5.0);
        is_following = false;
        is_opening = true;
    }
}

void callback(Robot* robot) {
    const double leftAngleDiff = angle_diff(robot->range, 1.3);
    const double rightAngleDiff = abs(angle_diff(robot->range, -1.3));

    const double buffer = 0.1;

    cout << "Robot's angle: " << robot->pos_t << endl;

    double range = robot->range;

    // if encounters a wall
    if (robot->range < 0.5) {
        // check if robot is facing west or east
        if (is_following) {
            cout << "Do following turn if detecting a wall" << endl;
            if (abs(angle_diff(robot->range, WEST)) < buffer) {
                turn_robot(robot, RIGHT);
            } else if (abs(angle_diff(robot->range, EAST)) < buffer) {
                turn_robot(robot, LEFT);
            }
        }

        cout << "Attempt turning" << endl;
        if (leftAngleDiff > rightAngleDiff) {
            turn_robot(robot, LEFT);
            if (is_following) {
                cout << "Following wall left!" << endl;
                wall_follow(robot);
            }
        } else { 
            turn_robot(robot, RIGHT);
            if (is_following) {
                cout << "Following wall right!" << endl;
                wall_follow(robot);
            }
        }
    } else {
        // check if there is an opening
        if (is_opening && !is_following) {
            cout << "Opening Detected!" << endl;
            turn_robot(robot, FORWARD);
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
