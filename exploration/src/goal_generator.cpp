#include <exploration/goal_generator.h>
#include <iostream>
#include "ros/ros.h"
#define PI 3.14159265359
using namespace std;
namespace exploration {
  double distance(double x1, double y1, double x2, double y2) {
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
  }

  int sign(double x) {
    return x >= 0 ? 1 : -1;
  }

  GoalGenerator::GoalGenerator(const double radius): radius_(radius) {
    // prepare the occupancy grid object with the values that will never change
    grid_.header.frame_id = "/map";

  }

  geometry_msgs::Pose2D GoalGenerator::generateGoal(geometry_msgs::PointStamped &a_point,
                                                    double robot_x, double robot_y,
                                                    costmap_2d::Costmap2D& map) {

    unsigned int a_point_grid_x, a_point_grid_y;
    const int radius_map = map.cellDistance(radius_);
    map.worldToMap(a_point.point.x, a_point.point.y, a_point_grid_x, a_point_grid_y);

    // Create descreteized circle

    std::vector<costmap_2d::MapLocation> polygon;
    const double dtheta = PI / 10;

    for(double theta = 0; theta < 2 * PI; theta += dtheta) {
      costmap_2d::MapLocation p;
      double x = radius_ * cos(theta) + a_point.point.x;
      double y = radius_ * sin(theta) + a_point.point.y;
      map.worldToMap(x, y, p.x, p.y);
      polygon.push_back(p);
    //
    }



    // get all cells within the cirle

    std::vector<costmap_2d::MapLocation> polygon_cells;
    map.convexFillCells(polygon, polygon_cells);



    // score each cell and find the max


    // prepare the occupancy grid
    grid_.info.resolution = map.getResolution();
    grid_.header.stamp = ros::Time::now();
    grid_.info.origin.position.x = a_point.point.x - radius_;
    grid_.info.origin.position.y = a_point.point.y + map.getResolution() - radius_;
    grid_.info.origin.position.z = 0.0;
    grid_.info.origin.orientation.w = 1.0;
    grid_.info.width = 2*radius_map + 2;
    grid_.info.height = 2*radius_map + 2;
    grid_.data.resize(grid_.info.width * grid_.info.height);

    // fill occpancy grid with unkown space (a value of -1)
    for(int i = 0; i < grid_.info.width * grid_.info.height; i++) {
      grid_.data[i] = -1;
    }

    // variables for storing the highest scoreing cell
    double max_score = -INFINITY;
    costmap_2d::MapLocation max_loc;

    // variables for converting the map location into an occupancy grid location
    const int x_offset = a_point_grid_x - radius_;
    const int y_offset = a_point_grid_y - radius_;

    // preperation for line scoreing
    unsigned int robot_grid_x, robot_grid_y;
    map.worldToMap(robot_x, robot_y, robot_grid_x, robot_grid_y);



    // find intersection points
    double dx, dy, dr, ans_x, ans_y, radical;
    costmap_2d::MapLocation p1, p2;
    dx = (double)robot_grid_x - (double)a_point_grid_x;
    dy = (double)robot_grid_y - (double)a_point_grid_y;


    dr = distance(dx, dy, 0, 0);


    radical = sqrt(radius_map * radius_map * dr * dr);



    ans_x = sign(dy) * dx * radical / (dr * dr);



    ans_y = abs(dy) * radical / (dr * dr);
    p1.x = ans_x + a_point_grid_x;
    p1.y = ans_y + a_point_grid_y;
    p2.x = -ans_x + a_point_grid_x;
    p2.y = -ans_y + a_point_grid_y;
    double map_x, map_y;
    map.mapToWorld(p1.x, p1.y, map_x, map_y);


    // find the point closer to the robot
    double d1, d2;
    costmap_2d::MapLocation *near_p, *far_p;
    d1 = distance(p1.x, p1.y, robot_grid_x, robot_grid_y);
    d2 = distance(p2.x, p2.y, robot_grid_x, robot_grid_y);
    if(d1 < d2) {
      near_p = &p1;
      far_p = &p2;
    } else {
      near_p = &p2;
      far_p = &p1;
    }

    // find line constants
    const double m = dy / dx;
    const double a = -m;
    const double b = 1;
    const double c = m*robot_grid_x - robot_grid_y;



    // loop through every cell in polygon and select the highest scoring one
    for(int i = 0; i != polygon_cells.size(); i++) {


      const int x = polygon_cells[i].x, y = polygon_cells[i].y;
      // don't score the attention point itself!
      if (x == a_point_grid_x && y == a_point_grid_y) continue;

      // find the score
      double score, obstical_score, dist_score, line_score;
      obstical_score = ((map.getCost(x, y) != 255) ? 100 - map.getCost(x, y) : -1);


      if (obstical_score == -1) {
        score = -1;
      } else if (obstical_score == 0 || obstical_score == 1) {
        score = -1;
      } else {
        const double optimal_dist = map.cellDistance(0.8);
        const double a_point_dist = sqrt(pow((double)x - a_point_grid_x, 2) +
                                         pow((double)y - a_point_grid_y, 2));
        dist_score = abs(radius_map - optimal_dist) - abs(a_point_dist - optimal_dist);
        dist_score /= abs(radius_map - optimal_dist);
        dist_score *= 100;

        //dist_score = 100 * (1.0 - (a_point_dist / radius_map));

        // calculate line score
        const double dist_near = distance(x, y, near_p->x, near_p->y);
        const double dist_far  = distance(x, y, far_p->x, far_p->y);
        if (dist_far < dist_near) {
          line_score = 0;
        } else {
          const double dist = abs(a*x + b*y + c) / sqrt(a*a + b*b);
          line_score = 100 * pow((1 - dist / radius_map), 2);
        }
        score = 0.50 * obstical_score + 0.05 * dist_score + 0.45 * line_score;
        //score = line_score;
        //score = dist_score;
      }







      //


      // store the score in the occupancy grid for visualization
      int index = (y - y_offset + radius_map - map.getResolution()/2) * grid_.info.width + (x - x_offset + radius_map - map.getResolution()/2);
      if(index > 0 && index < grid_.info.width * grid_.info.height) {

        grid_.data[index] = score;
        /*
        if(abs(x - p1.x) < 20 && abs(y - p1.y) < 20) {
          grid_.data[index] = 50;
        }
        if(abs(x - p2.x) < 20 && abs(y - p2.y) < 20) {
          grid_.data[index] = 50;
        }
        */
      }



      // select score if max
      if(score > max_score) {
        max_score = score;
        max_loc = polygon_cells[i];
      }
    }







    // nice little ros structure to store result
    geometry_msgs::Pose2D pose;
    map.mapToWorld(max_loc.x, max_loc.y, pose.x, pose.y);

    // find the angle
    double v_x = a_point.point.x - pose.x,
           v_y = a_point.point.y - pose.y;
    ROS_INFO("1) v_x: %f, v_y: %f", v_x, v_y);

    double mag = sqrt(v_x*v_x + v_y*v_y);
    ROS_INFO("mag: %f", mag);
    v_x /= mag;
    v_y /= mag;
    ROS_INFO("2) v_x: %f, v_y: %f", v_x, v_y);
    double yaw = acos(v_x) * sign(v_y);
    ROS_INFO(" acos(v_x): %f", acos(v_x));

    pose.theta = yaw;

    return pose;
  }
}
