
#include <iostream>
#include <math.h>

#include "robot.hh"

using std::cout;
using std::endl;

const double goal_x = 20.0;
const double goal_y = 0.0;
bool done = false;
double turnTotal = 0.0;

void
callback(Robot* robot)
{

    cout << endl;
    cout << "robot x = " << robot->pos_x << endl;
    cout << "robot y = " << robot->pos_y << endl;

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
        if (hit.range < 1.5) {
            if (hit.angle < 0.5 || hit.angle > (6.2 - 0.5)) {
                turn = true;
            }
        }
    }

    cout << "YAW " << robot->pos_t << endl;
   
    if (turn) {
	// check whether it has turned yet
	if (turnTotal == 0.0) {
	     turnTotal = 0.3;
	} else {
	     turnTotal += 0.01;
	}

	cout << "Turn by: " << turnTotal << endl;
        robot->set_vel(3.0);
	// set variable for accumulating turn
	robot->set_turn(turnTotal);
    } else {
	turnTotal = 0.0;
	// calculate angle between robot and goal coordinates
	double dtheta = atan2(dy, dx);
	cout << "dy = " << dy << endl;
	cout << "dx = " << dx << endl;
	cout << "Angle = " << -dtheta << endl;	
	robot->set_vel(10.0);
	robot->set_turn(-dtheta);
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
