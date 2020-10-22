#include <iostream>
#include <thread>
#include <math.h>

#include "robot.hh"
#include "viz.hh"

using std::cout;
using std::endl;

/* Robot State Properties */
bool is_following = false;
bool is_turning = false;
bool is_opening = false;

/* Provided so kindly by Trey */
float 
angle_avg(float* angv, size_t angc) 
{
    float sinsum = 0;
    float cossum = 0;

    for (size_t i = 0; i < angc; i++) {
        float ang = angv[i];
        float adj = ang < 0 ? (2 * M_PI) + ang : ang;
        sinsum += sinf(adj);
        cossum += cosf(adj);
    }

    float avg = atan2f(sinsum, cossum);
    return avg > M_PI ? (-2 * M_PI) + avg : avg;
}

/* Draws point based on the range, robot's position, and robot's angle */
void
draw_point(Robot* robot) 
{
    for (auto hit : robot->ranges) {
        // viz_hit(hit.range, hit.angle);
        viz_pos(robot->pos_x, robot->pos_y, robot->pos_t, hit.range);
        //cout << hit.range << "@" << hit.angle << endl;
    }
}

void
callback(Robot* robot)
{
    // const double leftAngleDiff = angle_diff(robot->range, 1.3);
    // const double rightAngleDiff = abs(angle_diff(robot->range, -1.3));
    draw_point(robot);
    if (robot->ranges.size() < 5) {
        return;
    }

    float left = clamp(0.0, robot->ranges[2].range, 4.0);
    float forward = clamp(0.0, robot->ranges[3].range, 4.0);
    float right = clamp(0.0, robot->ranges[4].range, 4.0);
    
    cout << "Left,Forward,Right Angles = "
         << left << ","
         << forward << ","
         << right << endl;

    float speed = forward - 1.0;
    float turn = clamp(-1.0, left - right, 1.0);

    // Cases to Consider:
    // If no forward, move north
    // If forward, move right
    // If left and forward, move right
    // If right and forward, move left
    if (forward > 1.2) {
      speed = 5;
      turn = 0;
    } else {
        if (forward < 1.2) {
            cout << "Turn right" << endl;
            speed = 0;
            turn = 2;

            if (left < 1 && right < 1) {
                turn = -2;
            }
        } else if (right < 1.5) {
            turn = 2;
        } else if (left < 1.5) {
            turn = -2;
        }
    }

    cout << "Speed,Turn = " << speed << "," << turn << endl;
    robot->set_vel(speed + turn, speed - turn);

    /*
    cout << "x,y,t = "
         << robot->pos_x << ","
         << robot->pos_y << ","
         << robot->pos_t << endl;
    robot->set_vel(robot->pos_t, -robot->pos_t);
    */
}

void
robot_thread(Robot* robot)
{
    robot->do_stuff();
}

int
main(int argc, char* argv[])
{
    cout << "making robot" << endl;
    Robot robot(argc, argv, callback);
    std::thread rthr(robot_thread, &robot);

    return viz_run(argc, argv);
}


// performs desired rotation on robot 
// void 
// turn_robot(Robot* robot, const MOVE type) 
// {
//     const double buffer = 0.1;
//     const double leftAngleDiff = angle_diff(robot->range, WEST);
//     const double rightAngleDiff = abs(angle_diff(robot->range, EAST));

//     is_turning = true;

//     switch(type) {
//         case LEFT:
//             if (leftAngleDiff > 0.01) {
//                 cout << leftAngleDiff << endl;
//                 robot->set_vel(-4.0, 4.0);

//                 if (leftAngleDiff > 0.005) {
//                     robot->set_vel(-1.0, 1.0);
//                 }
//             }
//             break;
//         case RIGHT:
//             if (rightAngleDiff > 0.01) {
//                 cout << rightAngleDiff << endl;
//                 robot->set_vel(4.0, -4.0);

//                 if (rightAngleDiff > 0.005) {
//                     robot->set_vel(1.0, -1.0);
//                 }
//             }
//             break;
//         case FORWARD:
//             robot->set_vel(3.0, 3.0);
//             is_turning = false;
//             break;
//     }

//     is_following = true;
// }

// void 
// wall_follow(Robot* robot) 
// {
//     gazebo::common::Time currTime = gazebo::common::Time::GetWallTime();
//     const double buffer = 0.1;
//     const double leftAngleDiff = angle_diff(robot->range, WEST);
//     const double rightAngleDiff = angle_diff(robot->range, EAST);

//     // if (leftAngleDiff < buffer || rightAngleDiff < buffer || robot->range < 1.7) {
//     //     robot->set_vel(5.0, 5.0);
//     // }

//     // spot opening
//     cout << "Range: " << robot->range << endl;
//     if(robot->range > 4.0) {
//         // move faster if facing north or south and enter opening
//         if (abs(angle_diff(robot->range, NORTH)) < buffer) {
//             cout << "Going straight-ish" << endl;

//             turn_robot(robot, FORWARD);
            
//             // wait before turning while robot moves forward
//             if (simTime == 0) {
//                 simTime = currTime.Double();
//             }

//             cout << "TIME:" << simTime - currTime.Double() << endl;
//             if (currTime.Double() - simTime > 2.5) {
//                 is_opening = true;
//             }

//             // decide to turn left or right
//             if (is_opening) {
//                 cout << "open" << endl;
//                 if (leftAngleDiff > 0) {
//                     turn_robot(robot, LEFT);
//                 } else if (rightAngleDiff < 0) {
//                     turn_robot(robot, RIGHT);
//                 }
//             }
//         }
//     } else {
//         turn_robot(robot, FORWARD);
//     }
// }