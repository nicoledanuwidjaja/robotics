#include <iostream>
#include <algorithm>
#include <thread>
#include <math.h>
#include <map>
#include <time.h>
#include <stdlib.h>

#include "viz.hh"
#include "robot.hh"

// #include "ekf.hh"
#include <bits/stdc++.h>

// using std::cout;
// using std::endl;
using namespace std;

#define PI 3.14159265

typedef pair<long,long> Posn;

#define LEFT    1
#define RIGHT   2
#define FORWARD 3

#define NORTH   0
#define NE      1
#define EAST    2
#define SE      3
#define SOUTH   4
#define SW      5
#define WEST    6
#define NW      7

#define OCC_FREE -0.5

double speed_modifier = 0.9;
time_t last_on_wall = 0;
int going_wf = FORWARD;
int prev_dir = NORTH;
double prev_ang = 0;
int turns = 0;

struct Cell {
    Posn loc; // location
    bool seen;  // has the cell been seen?
    double occupied; // how occupied it is. negative is free/empty, positive is a wall
    vector<Cell> neighbors; // adjacent neighbor cells
    Cell* parent; // node that connects to shortest path
    float goal_distance; // distance away from goal 
};

double x_pos_vals[20];
double y_pos_vals[20];

double x_mean = 0;
double x_var = 0;
double y_mean = 0;
double y_var = 0;

struct Pose {
    long xx;
    long yy;
    double theta;
    int dir;
};

struct Pose2 {
    double xx;
    double yy;
    double theta;
    int dir;
};

struct Err_corr {
    double x_coef;
    double x_int;
    double y_coef;
    double y_int;
};

map<Posn, Cell> o_grid; // global occupancy grid

Pose prev_pose = {0, 0, 0, 0};
Pose exp_pose = {0, 0, 0, 0};
Pose2 pose_change = {0, 0, 0, 0};
Pose2 acc_pose_change = {0, 0, 0, 0};

Err_corr err = {1, 0, 1, 0};

double last_velo_0 = 0;
double last_velo_1 = 0;
time_t last_cmd = 0;

// 700 by 700 approx. [-35, 35] x y (increments of 0.1)
// See the *10 in get_x_occ and get_y_occ

// raw values (not adjusted for resolution)
void update_pos_bufs(double xx, double yy) {
    int length = 20;
    for (int i = 1; i < length; i++) {
        x_pos_vals[i-1] = x_pos_vals[i];
        y_pos_vals[i-1] = y_pos_vals[i];
    }
    x_pos_vals[length - 1] = xx - acc_pose_change.xx;
    // cout << "EEEEX " <<  x_pos_vals[length - 1] << endl;
    y_pos_vals[length - 1] = yy - acc_pose_change.yy;
    // cout << "EEEEY " <<  y_pos_vals[length - 1] << endl;
}

void set_mean() {
    double acc_x = 0;
    double acc_y = 0;
    int i = 0;
    while (i < 20) {
        if (x_pos_vals[i] == 0 && x_pos_vals[i] == 0) {
            break;
        }
        acc_x = acc_x + x_pos_vals[i];
        acc_y = acc_y + y_pos_vals[i];
        i++;
    }
    // cout << "acc x " << acc_x << endl;
    // cout << "acc y " << acc_y << endl;
    x_mean = acc_x / double(i + 1);
    y_mean = acc_y / double(i + 1);
}

void set_var() {
    double acc_x = 0;
    double acc_y = 0;
    int i = 0;
    while (i < 20) {
        if (x_pos_vals[i] == 0 && x_pos_vals[i] == 0) {
            break;
        }
        acc_x = acc_x + pow((x_pos_vals[i] - x_mean), 2.0);
        acc_y = acc_y + pow((y_pos_vals[i] - y_mean), 2.0);
        i++;
    }
    x_var = acc_x / double(i);
    y_var = acc_y / double(i);
}

int get_dir(double thet) {
    if (abs(thet) <= (PI / 16)) {
        return NORTH;
    } else if (abs(thet) >= (15 * PI / 16)) {
        return SOUTH;
    } else if ((thet >= (7 * PI / 16)) && (thet <= (9 * PI / 16))) {
        return WEST;
    } else if ((thet <= (-7 * PI / 16)) && (thet >= (-9 * PI / 16))) {
        return EAST;
    } else if ((thet > (PI / 16)) && (thet < (7 * PI / 16))) {
        return NW;
    } else if ((thet > (9 * PI / 16)) && (thet < (15 * PI / 16))) {
        return SW;
    } else if ((thet < (-1 * PI / 16)) && (thet > (-7 * PI / 16))) {
        return NE;
    } else {
        return SE;
    }
}

int get_x_occ(double x_rob, double y_rob, double t_rob, double d_hit, double t_hit) {
    return ceil((10 * x_rob) + (10 * d_hit * (cos(t_rob + t_hit))));
}

int get_y_occ(double x_rob, double y_rob, double t_rob, double d_hit, double t_hit) {
    return ceil((10 * y_rob) + (10 * d_hit * (sin(t_rob + t_hit))));
}

int get_x_occ_free(double x_rob, double y_rob, double t_rob, double d_hit, double t_hit) {
    return ceil((10 * x_rob) + (9 * d_hit * (cos(t_rob + t_hit))));
}

int get_y_occ_free(double x_rob, double y_rob, double t_rob, double d_hit, double t_hit) {
    return ceil((10 * y_rob) + (9 * d_hit * (sin(t_rob + t_hit))));
}

double get_x_r(Robot* robot) {
    // cout << "ps x " << pose_change.xx << endl;
    // cout << "x-me " << x_mean << endl;
    // cout << "robot x " << robot->pos_x << endl;
    acc_pose_change.xx = acc_pose_change.xx + pose_change.xx;
    return x_mean + acc_pose_change.xx;
    // return x_mean;
    // return robot->raw_x;
}

double get_y_r(Robot* robot) {
    // cout << "ps y " << pose_change.yy << endl;
    // cout << "y-me " << y_mean << endl;
    // cout << "robot y " << robot->pos_y << endl;
    acc_pose_change.yy = acc_pose_change.yy + pose_change.yy;
    return y_mean + acc_pose_change.yy;
    // return y_mean;
    // return robot->raw_y;
}

void set_expected_raw(time_t new_time, Robot* robot) {
    int time_diff = new_time - last_cmd;
    double velo_avg = (last_velo_0 + last_velo_1) / 2;

    double dist = 0.9 * velo_avg * (time_diff / 10);

    pose_change.xx = ((dist * cos(robot->pos_t)));
    pose_change.yy = ((dist * sin(robot->pos_t)));
    exp_pose.xx = (ceil(prev_pose.xx + (10 * (dist * cos(robot->pos_t)))));
    exp_pose.yy = (ceil(prev_pose.yy + (10 * (dist * sin(robot->pos_t)))));
    exp_pose.theta = robot->pos_t;
    exp_pose.dir = get_dir(robot->pos_t);
}

// return if the expected xx/yy/theta match (within bounds)
bool here_odom(Robot *robot) {
    set_expected_raw(time(NULL), robot);
    if ((abs(ceil(10 * robot->pos_x) - exp_pose.xx) < 5) &&
        (abs(ceil(10 * robot->pos_y) - exp_pose.yy) < 5)) {
            prev_pose.xx = robot->pos_x;
            prev_pose.yy = robot->pos_y;
            prev_pose.theta = robot->pos_t;
            return true;
    } else {
        if ((abs(ceil(10 * get_x_r(robot)) - exp_pose.xx) < 5) &&
            (abs(ceil(10 * get_y_r(robot)) - exp_pose.yy) < 5)) {
            prev_pose.xx = ceil(10 * get_x_r(robot));
            prev_pose.yy = ceil(10 * get_y_r(robot));
        } else {
            // update_error_vals(robot);
            prev_pose.xx = ceil(10 * get_x_r(robot));
            prev_pose.yy = ceil(10 * get_y_r(robot));
        }
        prev_pose.theta = robot->pos_t;
        return false;
    }
}

void print_map(std::map<Posn, Cell> const &m) {
    for (auto it = m.begin(); it != m.end(); ++it) {
        std::cout << "Map entry: " << (*it).first.first << ":" << (*it).second.occupied << endl;
    }
}

void draw_current(long xx, long yy, double occ_add) {
    double occ_val = occ_add;
    std::map<Posn, Cell>::iterator iti = o_grid.find(make_pair(xx, yy));
    if (iti != o_grid.end()) {
        occ_val = clamp(-5, iti->second.occupied + occ_add, 5);
        o_grid.erase(iti);
    }
    struct Cell ci;
    ci.seen = true;
    ci.occupied = occ_val;
    o_grid.emplace(make_pair(xx, yy), ci); // emplace checks if key is unique
    // cout << "Pair x: " << xx << endl;
    // cout << "Pair y: " << yy << endl;
    viz_hit(xx, yy, occ_val);
    print_map(o_grid);
}

void bresenham_free(long x1, long y1, long x2, long y2) {
    int m_new = 2 * (y2 - y1); 
    int slope_error_new = m_new - (x2 - x1); 
    
    for (int x = x1, y = y1; x <= x2; x++) { 
        draw_current(x, y, OCC_FREE);

        // Add slope to increment angle formed 
        slope_error_new += m_new; 

        // Slope error reached limit, time to 
        // increment y and update slope error. 
        if (slope_error_new >= 0) { 
            y++; 
            slope_error_new -= 2 * (x2 - x1); 
        } 
    }
}

void bresenham_free_big(long x1, long y1, long x2, long y2) {
    int m_new = 2 * (x2 - x1); 
    int slope_error_new = m_new - (y2 - y1); 
    
    for (int x = x1, y = y1; y <= y2; y++) { 
        draw_current(x, y, OCC_FREE);

        // Add slope to increment angle formed 
        slope_error_new += m_new; 

        // Slope error reached limit, time to 
        // increment y and update slope error. 
        if (slope_error_new >= 0) { 
            x++; 
            slope_error_new -= 2 * (y2 - y1); 
        } 
    }
}

void bresenham_free_rev_big(long x1, long y1, long x2, long y2) {
    int m_new = 2 * (x2 - x1); 
    int slope_error_new = m_new - (y1 - y2); 
    
    for (int x = x2, y = y2; y <= y1; y++) { 
        draw_current(x, y, OCC_FREE);

        // Add slope to increment angle formed 
        slope_error_new += m_new; 

        // Slope error reached limit, time to 
        // increment y and update slope error. 
        if (slope_error_new >= 0) { 
            x--; 
            slope_error_new -= 2 * (y1 - y2); 
        } 
    }
}

void bresenham_free_rev(long x1, long y1, long x2, long y2) {
    int m_new = 2 * (y1 - y2); 
    int slope_error_new = m_new - (x2 - x1); 
    
    for (int x = x2, y = y2; x >= x1; x--) { 
        draw_current(x, y, OCC_FREE);

        // Add slope to increment angle formed 
        slope_error_new += m_new; 

        // Slope error reached limit, time to 
        // increment y and update slope error. 
        if (slope_error_new >= 0) { 
            y++; 
            slope_error_new -= 2 * (x2 - x1); 
        } 
    }
}

void bs_line_free(long x1, long y1, long x2, long y2) {
    if (x1 != x2 || y1 != y2) {
        float slo = (float(y2) - float(y1)) / (float(x2) - float(x1));
        float sloREV = (float(y1) - float(y2)) / (float(x1) - float(x2));
        if (x1 <= x2 && y1 <= y2 && slo <= 1.0) {
            bresenham_free(x1, y1, x2, y2);
        } else if (x1 <= x2 && y1 <= y2 && slo > 1.0) {
            bresenham_free_big(x1, y1, x2, y2);
        } else if (x1 >= x2 && y1 >= y2 && sloREV <= 1.0) {
            bresenham_free(x2, y2, x1, y1);
        } else if (x1 >= x2 && y1 >= y2 && sloREV > 1.0) {
            bresenham_free_big(x2, y2, x1, y1);
        } else if (x1 <= x2 && y1 >= y2 && (slo < 0 && slo >= -1)) {
            bresenham_free_rev(x1, y1, x2, y2);
        } else if (x1 <= x2 && y1 >= y2 && (slo < -1)) {
            bresenham_free_rev_big(x1, y1, x2, y2);
        } else if (x1 >= x2 && y1 <= y2 && (sloREV < 0 && sloREV >= -1)) {
            bresenham_free_rev(x2, y2, x1, y1);
        } else if (x1 >= x2 && y1 <= y2 && (sloREV < -1)) {
            bresenham_free_rev_big(x2, y2, x1, y1);
        }
    } else {
        draw_current(x1, y1, OCC_FREE);
    }
}

void map_make(Robot* robot) {
    // for (std::map<Posn, Cell>::iterator it=o_grid.begin(); it!=o_grid.end(); ++it) {
    //     cout << it->first.first << " , " << it->first.second << " => " << it->second.occupied << '\n';
    // }
    update_pos_bufs(robot->pos_x, robot->pos_y);
    // update_pos_bufs(robot->raw_x, robot->raw_y);

    set_mean();
    set_var();

    double xr = get_x_r(robot);
    double yr = get_y_r(robot);
    for (auto hit : robot->ranges) {
        // cout << hit.range << " @ " << hit.angle << endl;
        if (hit.range < 100) {
            long xx = get_x_occ(xr, yr, robot->pos_t, hit.range, hit.angle);
            long yy = get_y_occ(xr, yr, robot->pos_t, hit.range, hit.angle);

            long xx2 = get_x_occ_free(xr, yr, robot->pos_t, hit.range, hit.angle);
            long yy2 = get_y_occ_free(xr, yr, robot->pos_t, hit.range, hit.angle);
            // bresenham + not walls
            bs_line_free((ceil(10 * xr)), (ceil(10 * yr)), xx2, yy2);

            draw_current(xx, yy, 0.9);
        } else { // infinite range --> range goes to 2 meters.
            long xx = get_x_occ_free(xr, yr, robot->pos_t, 2.0, hit.angle);
            long yy = get_y_occ_free(xr, yr, robot->pos_t, 2.0, hit.angle);
            // bresenham + not walls
            bs_line_free((ceil(10 * xr)), (ceil(10 * yr)), xx, yy);
        }
    }
    
    draw_robot_pose((ceil(10 * xr)), (ceil(10 * yr)), robot->pos_t);
}

void drive_forward(Robot* robot, double spd) {
    int current_dir = get_dir(robot->pos_t);
    if (prev_dir != current_dir) {
        double ang_diff = robot->pos_t - prev_ang;
        robot->set_vel(spd + ang_diff, spd - ang_diff);
        last_velo_0 = spd + ang_diff;
        last_velo_1 = spd - ang_diff;
    } else {
        robot->set_vel(spd, spd);
        last_velo_0 = spd;
        last_velo_1 = spd;
        prev_dir = floor(get_dir(robot->pos_t) / 2) * 2;
        prev_ang = robot->pos_t;
    }
    return;
}

void move_robot(Robot* robot) {
    double min_range = 10;
    // float count = 0;
    for (auto hit : robot->ranges) {
        if (hit.range < 100 && (hit.angle <= (PI/4)) && hit.angle > (-1 * PI/4)) {
            // avg_range = (avg_range * count + hit.range) / (count + 1.0);
            if (hit.range < min_range) {
                min_range = hit.range;
            }
            // count = count + 1.0;
        }
    }

    if (min_range < 1.0) {
        robot->set_vel(speed_modifier* 2.0, speed_modifier* -3.0);
        last_velo_0 = speed_modifier* 2.0;
        last_velo_1 = speed_modifier* -3.0;
        prev_dir = floor(get_dir(robot->pos_t) / 2) * 2;
        prev_ang = robot->pos_t;
        last_cmd = time(NULL);
        last_on_wall = time(NULL);
        return;
    }

    if ((robot->ranges[5].range < 1.5 || robot->ranges[1].range < 1.5) &&
            robot->ranges[2].range > 100 && robot->ranges[3].range > 100 &&
            robot->ranges[4].range > 100) {
                drive_forward(robot, 4.0 * speed_modifier);
                last_cmd = time(NULL);
                last_on_wall = time(NULL);
                return;
            }

    if (min_range > 1.9 && (time(NULL) - last_on_wall) < 5) {
        robot->set_vel(speed_modifier* -2.0, speed_modifier *3.0);
        last_velo_0 = speed_modifier* -2.0;
        last_velo_1 = speed_modifier* 3.0;
        prev_dir = floor(get_dir(robot->pos_t) / 2) * 2;
        prev_ang = robot->pos_t;
    } else {
        drive_forward(robot, 4.0 * speed_modifier);
    }

    last_cmd = time(NULL);
    return;
}

void acc_pos(Robot* robot) {
    if (prev_pose.xx != 0 && prev_pose.yy != 0) {
        // here_odom(robot);
        prev_pose.theta = robot->pos_t;
        set_expected_raw(time(NULL), robot);
    } else {
        acc_pose_change.xx = robot->pos_x;
        acc_pose_change.yy = robot->pos_y;
        prev_pose.xx = (ceil(10 * robot->pos_x));
        prev_pose.yy = (ceil(10 * robot->pos_y));
        prev_pose.theta = robot->pos_t;
        prev_pose.dir = get_dir(robot->pos_t);
        set_expected_raw(time(NULL), robot);
    }
}

/* Add neighbors for a given cell */
void add_neighbors(Cell c) {
    // boundaries of map: [-35, 35] (x, y)
    if (c.neighbors.empty()) {
        cout << "Loc: " << c.loc.first << endl;
        cout << "second Loc: " << c.loc.second << endl;
        // north neighbor
        if (c.loc.second < 35) {
            struct Cell neighbor;
            neighbor.loc = make_pair(c.loc.first, c.loc.second + 1);
            c.neighbors.push_back(neighbor);
        }
        // west neighbor
        if (c.loc.first > -35) {
            struct Cell neighbor;
            neighbor.loc = make_pair(c.loc.first - 1, c.loc.second);
            c.neighbors.push_back(neighbor);
        }
        // south neighbor
        if (c.loc.second > -35) {
            struct Cell neighbor;
            neighbor.loc = make_pair(c.loc.first, c.loc.second - 1);
            c.neighbors.push_back(neighbor);
        }
        // east neighbor
        if (c.loc.first < 35) {
            struct Cell neighbor;
            neighbor.loc = make_pair(c.loc.first + 1, c.loc.second);
            c.neighbors.push_back(neighbor);
        }
    }
    return;
}

/* calculate heuristic distance cost */
float get_heur(pair<int, int> src, pair<int, int> dest) {
    return sqrtf(pow((src.first - dest.first), 2) + pow((src.second - dest.second), 2));
}

/* Compare cells based on their f-scores */
bool compare_dist(const Cell& c1, const Cell& c2) {
    return c1.goal_distance < c2.goal_distance;
}

/* A* searching algorithm */
void find_path(pair<int, int> src, pair<int, int> dest) {
    int start_x = src.first;
    int start_y = src.second;
    Posn curr = make_pair(start_x, start_y);
    Cell start_cell = o_grid.find(curr)->second;
    start_cell.goal_distance = get_heur(src, dest);
    cout << "Goal Distance: " << start_cell.goal_distance;
    add_neighbors(start_cell);
    // stores open list of cell properties and f-score
    // f = heuristic = g + h (row/column indices)
    vector<Cell> final_list;
    vector<Cell> free_list;
    free_list.insert(start_cell);
    bool reached_dest = false;

    while (!free_list.empty()) {
        std::sort(free_list.begin(), free_list.end(), compare_dist);
        // obtain value in free_list with the lowest f score
        auto min_cell = std::min_element(free_list.begin(), free_list.end(), compare_dist);
        // final_list = free_list.begin();
        // free_list.erase(final_list);

        // src.first = final_list.second.first;
        // src.second = final_list.second.second;
        // final_list[src.first][src.second] = true;

        // for (auto it = free_list.begin(); it != free_list.end(); it++) {
        //     auto item = *it;
        //     if (get_heur(item) <= get_heur(final_list[src.first][src.second])) {
        //         final_list[src.first][src.second] = item;
        //     }
        // }
    }
}

void callback(Robot* robot) {
    cout << "------------------------------" << endl;
    cout << "stamp " << robot->stamp << endl;
    cout << "pos X " << robot->pos_x << endl;
    cout << "pos Y " << robot->pos_y << endl;
    cout << "pos T " << robot->pos_t << endl;

    acc_pos(robot);

    map_make(robot);

    if (robot->ranges.size() < 5) {
        return;
    }

    move_robot(robot);

    pair<int, int> start = make_pair(robot->pos_x, robot->pos_y);
    pair<int, int> end = make_pair(20, 0);

    find_path(start, end);
}

void robot_thread(Robot* robot) {
    robot->do_stuff();
}

int main(int argc, char* argv[]) {
    cout << "making robot" << endl;
    Robot robot(argc, argv, callback);
    std::thread rthr(robot_thread, &robot);

    return viz_run(argc, argv);
}
