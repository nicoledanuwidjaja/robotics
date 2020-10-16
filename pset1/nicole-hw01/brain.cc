
#include <iostream>
#include <math.h>

#include "robot.hh"

using std::cout;
using std::endl;

const double goal_x = 20.0;
const double goal_y = 0.0;
bool done = false;

void
callback(Robot* robot)
{
    cout << "Robot Position = (" << robot->pos_x << "," << robot->pos_y << ")" << endl;
    cout << "Yaw = " << robot->pos_t << endl;
 
    // check if robot has arrived at destination
    double dx = goal_x - robot->pos_x;
    double dy = goal_y - robot->pos_y;
    if (abs(dx) < 0.75 && abs(dy) < 0.75) {
        cout << "we win!" << endl;
        robot->set_vel(0.0);
        robot->set_turn(0.0);
        robot->done();
        return;
    }

    bool turn = false;

    // check if laser scanner detects object in proximity, then turn
    for (LaserHit hit : robot->hits) {	
        if (hit.range < 2.0) {
	    cout << "RANGE: " << hit.range << endl;
	    cout << "ANGLE: " << hit.angle << endl;
            if (hit.angle < 0.69 && hit.angle > -0.77) {
                turn = true;
            }
        }
    }
   
    if (turn) {
	cout << "TURNING" << endl;
	robot->set_vel(5.0);
	robot->set_turn(0.5);
    } else {
	// calculate angle between robot and goal coordinates
	double goalAngle = atan2(dy, dx);
	double currAngle = goalAngle - robot->pos_t;

	cout << "Distance = (" << dx << "," << dy << ")" << endl;
	cout << "Angle = " << -currAngle << endl;	
	robot->set_vel(6.5);
	
	if(currAngle >= 0.1) {
	     currAngle = 0.5;
	} else if (currAngle < -0.1) {
	     currAngle = -0.5;
	}

	robot->set_turn(-currAngle);
    }
}

int
main(int argc, char* argv[])
{
    cout << "making robot" << endl;
    Robot robot(argc, argv, callback);
    robot.do_stuff();

    return 0;
}
