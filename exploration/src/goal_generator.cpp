#include <exploration/goal_generator.h>
#include "ros/ros.h"
#define PI 3.14159265359
using namespace std;
namespace exploration {

  GoalGenerator::GoalGenerator(const int radius): radius_(radius) {
    // prepare the occupancy grid object with the values that will never change
    grid_.header.frame_id = "/map";
    grid_.info.width = 2*radius_;
    grid_.info.height = 2*radius_;
    grid_.data.resize(grid_.info.width * grid_.info.height);
  }

  geometry_msgs::Pose2D GoalGenerator::generateGoal(geometry_msgs::PointStamped &a_point,
                                                    double robot_x, double robot_y,
                                                    costmap_2d::Costmap2D& map) {

    // Create descreteized circle
    ROS_INFO("creating circle");
    std::vector<costmap_2d::MapLocation> polygon;
    const double dtheta = PI / 10;

    for(double theta = 0; theta < 2 * PI; theta += dtheta) {
      costmap_2d::MapLocation p;
      double x = radius_ * cos(theta) + a_point.point.x;
      double y = radius_ * sin(theta) + a_point.point.y;
      map.worldToMap(x, y, p.x, p.y);
    }

    // get all cells within the cirle
    ROS_INFO("filling circle");
    std::vector<costmap_2d::MapLocation> polygon_cells;
    map.convexFillCells(polygon, polygon_cells);

    // score each cell and find the max
    ROS_INFO("scoring");
    unsigned int a_point_grid_x, a_point_grid_y;
    double a_point_x = a_point.point.x;
    double a_point_y = a_point.point.y;
    map.worldToMap(a_point_x, a_point_y, a_point_grid_x, a_point_grid_y);

    // prepare the occupancy grid
    grid_.info.resolution = map.getResolution();
    grid_.header.stamp = ros::Time::now();
    grid_.info.origin.position.x = a_point.point.x - map.getResolution() / 2;
    grid_.info.origin.position.y = a_point.point.y - map.getResolution() / 2;
    grid_.info.origin.position.z = 0.0;
    grid_.info.origin.orientation.w = 1.0;

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

    // loop through every cell in polygon and select the highest scoring one
    for(int i = 0; i != polygon_cells.size(); i++) {
      // find the score
      const int x = polygon_cells[i].x, y = polygon_cells[i].y;
      double score;
      score = map.getCost(x, y);
      const double a_point_dist = sqrt(pow((double)x - a_point_grid_x, 2) +
                                       pow((double)y - a_point_grid_y, 2));
      score -= a_point_dist;

      // store the score in the occupancy grid for visualization
      int index = (y - y_offset) * grid_.info.width + (x - x_offset);
      grid_.data[index] = score;

      // select score if max
      if(score > max_score) {
        max_score = score;
        max_loc = polygon_cells[i];
      }
    }

    ROS_INFO("found max score of %f", max_score);

    // nice little ros structure to store result
    geometry_msgs::Pose2D pose;
    pose.x = max_loc.x;
    pose.y = max_loc.y;
    pose.theta = 0;
    return pose;
  }
}
