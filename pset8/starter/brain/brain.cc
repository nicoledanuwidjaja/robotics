#include <iostream>
#include <math.h>
#include "viz.hh"
#include "robot.hh"
#include "cam.hh"

using std::cout;
using std::endl;

std::mutex mx2;

#define PI 3.14159265

std::vector<std::vector<bool>> hit_lookup;
std::vector<float> coll        = {0, 0, 0};
std::vector<float> drive_shift = {0, 0, 0};
std::vector<float> save_guess  = {0, 0, 0};
std::vector<float> guess       = {0, 0, 0};
std::vector<float> offset      = {0, 0, 0};
std::vector<float> last_guess  = {0, 0, 0};
std::vector<float> thread_pos  = {0, 0, 0};

std::deque<std::vector<int>> target_path;
std::deque<std::vector<int>> target_path_thread;
std::vector<int> target_point = {};

bool has_path    = false;
bool hit_drive   = false;
bool coll_delete = false;
bool left_follow = false;
bool right_follow = false;
bool searching = false;
bool at_goal = false;
bool new_goal = false;

// time variables
float time_delt = 0;
float time_delta = 0;

// tuned values
float grid_div = 0.2f; 
float input_scalar = 1.05f;
float predict_weight = 20.0f;
float hit_weight = 0.1f;
int grid_offset = 28;
int setup_c = 0;
int check_count2 = 0;
int check_limit2 = 50;

// robot wheel values
float slip_constant = 2.0f * 0.68f;

float chassis_dy   = 0.216f;
float wheel_radius = 0.050f;
float wheel_width  = 0.040f;
float wheel_y0     = (chassis_dy * 0.5f + wheel_width * 0.6f + 0.010f);
float wheel_tread  = wheel_y0 * 2;

float last_time = 0;

std::vector<float> last_drive = {0, 0};
std::vector<float> guess_diff = {0, 0};

int size = std::floor((float(grid_offset) * 2.0)/grid_div);
float radius = float(grid_offset);

std::vector<std::vector<float>> tile_grid(size, std::vector<float>(size, -1));
std::vector<std::vector<float>> tile_grid_thread(size, std::vector<float>(size, -1));

double clamp_value(double xmin, double xx, double xmax) {
    if (xx < xmin) return xmin;
    if (xx > xmax) return xmax;
    return xx;
}

void print_arr(std::vector<float> v) {
  for (auto elem : v) {
    cout << elem << " ";
  }
  cout << endl;
}

/****************************************
 *  DRAWING 
 ****************************************/

// void draw_point(float x, float y, float r, float g, float b) {

//   float x_delt = radius - x;
//   float y_delt = radius - y;

//   float ang = atan2(y_delt, x_delt);
//   float dist = sqrt((y_delt * y_delt) + (x_delt * x_delt));

//   viz_hit(-dist/float(radius) * 1.9f, ang, r, g, b);
// }

// void add_point(float x, float y, float r, float g, float b, std::vector<std::vector<float>>* point_list) {
//   float min = FLT_MAX;
  
//   for(const std::vector<float>& master: *point_list) {
//     float x_dlt = (x - master[0]);
//     float y_dlt = (y - master[1]);

//     float dist = x_dlt * x_dlt + y_dlt * y_dlt;
    
//     if (dist < min) {
//       min = dist;
//     }
//   }

//   if (min > 0.5f) {
//     std::vector<float> out = {x, y};
//     point_list->emplace_back(out);
//     draw_point(x, y, r, g, b);
//   }
// }

// void draw_path (std::deque<std::vector<int>> d_path) {
//   for (int i = 0; i < d_path.size(); ++i)
//   {
//     std::vector<int> point = d_path[i];
//     draw_point(float(point[0] * grid_div), float(point[1] * grid_div), 0.0f, 1.0f, 1.0f);
//   }
// }

/****************************************
 *  MOTION 
 ****************************************/

// apply a motion to the robot and predict the outcome
void cmd_vel(Robot* robot, float vx, float vy, bool print) {
  float tot = guess_diff[0] * guess_diff[0] + guess_diff[1] * guess_diff[1];
  float coll_tot = coll[0] * coll[0] + coll[1] * coll[1];

  float pred_vec = guess_diff[0] * coll[0] + coll[1] * guess_diff[1];
  float diff_scal = ((guess_diff[0] - coll[0]) * (guess_diff[0] - coll[0])) + ((guess_diff[1] - coll[1]) * (guess_diff[1] - coll[1]));

  float d_s = 0.2 * clamp_value(1, diff_scal / 0.01f, 3);

  if (coll_delete && ((abs(pred_vec) > 0 && coll_tot > 0) || abs(coll[2] - guess_diff[2]) > 0.001)){
    float a1 = atan2(coll[1], coll[0]);
    float a2 = atan2(guess_diff[1], guess_diff[0]);
    float tang = (a1 - a2);

    if (guess_diff[2] > PI) {
      guess_diff[2] -= PI * 2;
    } else if (guess_diff[2] < -PI) {
      guess_diff[2] += PI * 2;
    }

    if (tang > PI) {
      tang -= PI * 2;
    } else if (tang < -PI) {
      tang += PI * 2;
    }

    cout << "@@@@@@@@@@@@@@ " << tang << " " << drive_shift[0] << " " << drive_shift[1]<< endl;
    if ((tang > 0.1 && abs(last_drive[0] + last_drive[1]) / 2.0f > 0.1f) || coll[2] < guess_diff[2]) {
      drive_shift[1] += d_s;
      drive_shift[0] -= d_s;
    }

    if ((tang < -0.1 && abs(last_drive[0] + last_drive[1]) / 2.0f > 0.1f ) || coll[2] > guess_diff[2]) {
      drive_shift[1] -= d_s;
      drive_shift[0] += d_s;
    }

    cout << guess_diff[0] << " " << guess_diff[1] << endl;
    cout << coll[0] << " " << coll[1] << endl;
    
    last_drive = {0, 0};
    cout << "@@@@@@@@@@@@@@ " << tang << " " << drive_shift[0] << " " << drive_shift[1]<< endl;

    if (tot < coll_tot - 0.001 || pred_vec < 0) {
        drive_shift[0] -= d_s;
        drive_shift[1] -= d_s;
    } else if (tot > coll_tot + 0.001) {
        drive_shift[0] += d_s;
        drive_shift[1] += d_s;
    }
    cout << "@@@@@@@@@@@@@@ " << tang << " " << drive_shift[0] << " " << drive_shift[1] << endl;
  }

  last_drive = {last_drive[0] + vx, last_drive[1] + vy};

  cout << "vx:" << vx << " vy:" << vy << endl;
  cout << "tot:" << tot << " ct:" << coll_tot << " t:" << coll_delete << " d" << abs(time_delta) << " l:" << pred_vec  << " " << guess_diff[2] << " as:" << coll[2] << endl;

  if (coll_delete) {
    coll_delete = false;
    coll[0] = 0;
    coll[1] = 0;
    coll[2] = 0; 
  }

  if (abs(time_delta) < 1000) {
    coll[0] +=  ((vx + vy) / 2.0f * time_delta * wheel_radius * PI * slip_constant * cos(guess[2]));
    coll[1] +=  ((vx + vy) / 2.0f * time_delta * wheel_radius * PI * slip_constant * sin(guess[2]));
    coll[2] += -((vx - vy) / 2.0f * time_delta * wheel_radius * PI * slip_constant / (wheel_y0 * PI * 2));
  }

  drive_shift[0] = clamp_value((-10.0f * 0.2f) / input_scalar,  drive_shift[0], (10.0f * 0.2f) / input_scalar);
  drive_shift[1] = clamp_value((-10.0f * 0.2f) / input_scalar,  drive_shift[1], (10.0f * 0.2f) / input_scalar);

  robot->set_vel(vx / input_scalar - drive_shift[0], vy / input_scalar - drive_shift[1]);
}

// apply a value shift in a plus pattern arround the given x, y position
void apply_adj(int x, int y, float v, std::vector<std::vector<float>> &grid) {
  int ax = x + 1;
  int ay = y + 0;

  int bx = x - 1;
  int by = y + 0;

  int cx = x + 0;
  int cy = y + 1;

  int dx = x + 0;
  int dy = y - 1;

  if (ax >= 0 && ay >= 0 && ax < grid.size() && ay < grid[0].size()) {
    if (grid[ax][ay] == -1) {
      grid[ax][ay] = 0;
    }
    grid[ax][ay] += v;
    grid[ax][ay] = clamp_value(-0.99, grid[ax][ay], 2.0);
  }

  if (bx >= 0 && by >= 0 && bx < grid.size() && by < grid[0].size()) {
    if (grid[bx][by] == -1) {
      grid[bx][by] = 0;
    }
    grid[bx][by] += v;
    grid[bx][by] = clamp_value(-0.99, grid[bx][by], 2.0);
  }

  if (cx >= 0 && cy >= 0 && cx < grid.size() && cy < grid[0].size()) {
    if (grid[cx][cy] == -1) {
      grid[cx][cy] = 0;
    }
    grid[cx][cy] += v;
    grid[cx][cy] = clamp_value(-0.99, grid[cx][cy], 2.0);
  }

  if (dx >= 0 && dy >= 0 && dx < grid.size() && dy < grid[0].size()) {
    if (grid[dx][dy] == -1) {
      grid[dx][dy] = 0;
    }
    grid[dx][dy] += v;
    grid[dx][dy] = clamp_value(-0.99, grid[dx][dy], 2.0);
  }
}

// is the tile adjacent to a wall
bool is_adj(int x, int y, float v, std::vector<std::vector<float>> &grid, int d) {

  int width = grid.size();
  int height = grid[0].size();

  for (int xs = -d; xs <= d; ++xs)
  {
    for (int ys = -d; ys <= d; ++ys)
    {
      int tx = x + xs;
      int ty = y + ys;
      if (tx >= 0 && ty >= 0 && tx < width && ty < height) {
        if (tile_grid[tx][ty] > v) {
          return true;
        }
      }
    }
  }
  return false;
}

// update the costmap
bool plot_point_b(int x, int y, bool add, std::vector<std::vector<float>> &grid, float draw) {

  if (x < 0 || y < 0 || x >= grid.size() || y >= grid[0].size()) {
    cout << x << " " << y << endl;
    return true;
  }

  if (grid[x][y] == -1) {
    grid[x][y] = 0;
  }

  if (grid[x][y] < -0.9) {
    grid[x][y] = -0.9;
  }

  if (add) {
    grid[x][y] += 0.45f;
    apply_adj(x, y, 0.1, grid);
  } else {
    grid[x][y] -= 0.45f;
    apply_adj(x, y, -0.1, grid);;
  }

  grid[x][y] = clamp_value(-0.99, grid[x][y], 2.0);

  // if (draw) {
  //   draw_point((x * grid_div), (y * grid_div), clamp_value(0, grid[x][y], 1.0f) / 1.0f, 0, 0);
  // }
  return false;
}

// check the value fo the given tile and the adjacent tiles if denoted
bool check_val(int x, int y, float val, bool adj, std::vector<std::vector<float>> &grid, int range) {
  if (x < 0 || y < 0 || x >= grid.size() || y >= grid[0].size()) {
    return true;
  }

  if (grid[x][y] > val || (adj && is_adj(x, y, val, grid, range))){
    return true;
  }

  return false;
}







/*
To view the camera image in time, you could press CTRL-T in Gazebo
, choosing the Topic-"~/tankbot0/tankbot/camera_sensor/link/camera/image", 
then a Image Window will pop up, in order to view the Image in time.
*/

void
callback(Robot* robot)
{
    cout << "here2" <<endl;
    cam_show(robot->frame);

    robot->set_vel(0.0, 0.0);
}

int
main(int argc, char* argv[])
{
    cout << "here1" <<endl;
    cam_init();

    cout << "making robot" << endl;
    Robot robot(argc, argv, callback);
    robot.do_stuff();

    return 0;
}