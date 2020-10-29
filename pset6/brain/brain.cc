#include <iostream>
#include <thread>
#include <math.h>
#include "robot.hh"
#include "viz.hh"

using std::cout;
using std::endl;


/* Occupancy Grid Mapping map representation */
Cell map[280][280];

/* Robot State Properties */
double start_time = -1;
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

/* Calculate position and probability of obstacle detection */
void
calculate_prob(float dist, float angle, float pos_x, float pos_y) {
    // calculate hit position
    float dx = (dist * 4) * cos(angle);
    float dy = (dist * 4) * sin(angle);
    int normPosX = 180 + pos_x;
    int normPosY = 180 + pos_y;
    int x = (int) round(normPosX + dx);
    int y = (int) round(normPosY + dy);
    cout << "Plot x, y: " << x << " " << y << endl;

    // add position to map
    Cell hit_cell;
    hit_cell.num_hits = map[x][y].num_hits + 1;
    hit_cell.prob = map[x][y].prob + 0;
    cout << "Cell Prob: " << hit_cell.prob << endl;
    map[x][y] = hit_cell;
}

/* Draws point based on the range, robot's position, and robot's angle */
void
render_point(Robot* robot) 
{
    cout << "Render!" << endl;
    for (auto hit : robot->ranges) {
        //cout << hit.range << "@" << hit.angle << endl;
        float pos_x = robot->pos_x;
        float pos_y = robot->pos_y;
        float angle = robot->pos_t;
        // cout << "Robot points: " << pos_x << " " << pos_y << endl;
        float dist = hit.range;
        // assign a Bayesian probability value based on length of distance
        double prob = 0;
        if (dist < 2) {
            cout << "Object detected, I think" << endl;
            prob = 0.9;
        } else {
            cout << "Clear" << endl;
            prob = 0.0;
            dist = 2;
        }

        // // mark all spaces within range as free
        // for (int i = 0; i < dist; i++) {
        //     int curr_x = (int) round(sin(angle) * i);
        //     int curr_y = (int) round(cos(angle) * i);
        //     int x = (int) round(pos_x + curr_x);
        //     int y = (int) round(pos_y + curr_y);

        //     // add position to map
        //     Cell curr_cell;
        //     curr_cell.num_hits = map[x][y].num_hits + 1;
        //     curr_cell.prob = 0;
        //     cout << "Curr Cell Prob: " << curr_cell.prob << endl;
        //     map[x][y] = curr_cell;
        // }

        calculate_prob(dist, angle, pos_x, pos_y);
    }
}

void
callback(Robot* robot)
{
    // const double leftAngleDiff = angle_diff(robot->range, 1.3);
    // const double rightAngleDiff = abs(angle_diff(robot->range, -1.3));
    render_point(robot);
    
    if (robot->stamp - start_time > 2 && robot->stamp < 1000000) {
        cout << "I reached here" << endl;
        start_time = robot->stamp;
        viz_pos(map);
    }
    
    // cout << "Left,Forward,Right Angles = "
    //      << left << ","
    //      << forward << ","
    //      << right << endl;

    if (robot->ranges.size() < 5) {
        return;
    }

    float lft = clamp(0.0, robot->ranges[2].range, 2.0);
    float fwd = clamp(0.0, robot->ranges[3].range, 2.0);
    float rgt = clamp(0.0, robot->ranges[4].range, 2.0);
    float spd = fwd - 1.0;
    float trn = clamp(-1.0, lft - rgt, 1.0);

    if (fwd < 1.2) {
      spd = 0;
      trn = 1;
    }
    
    robot->set_vel(spd + trn, spd - trn);

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