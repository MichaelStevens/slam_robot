#include <exploration/goal_generator.h>
#include <iostream>
#include "ros/ros.h"
#define PI 3.14159265359
using namespace std;
namespace exploration {

  GoalGenerator::GoalGenerator(const double radius): radius_(radius) {
    // prepare the occupancy grid object with the values that will never change
    grid_.header.frame_id = "/map";

  }

  geometry_msgs::Pose2D GoalGenerator::generateGoal(geometry_msgs::PointStamped &a_point,
                                                    double robot_x, double robot_y,
                                                    costmap_2d::Costmap2D& map) {

    unsigned int a_point_grid_x, a_point_grid_y;
    double a_point_x = a_point.point.x;
    double a_point_y = a_point.point.y;
    map.worldToMap(a_point_x, a_point_y, a_point_grid_x, a_point_grid_y);
    // Create descreteized circle
    ROS_INFO("creating circle");
    std::vector<costmap_2d::MapLocation> polygon;
    const double dtheta = PI / 10;

    for(double theta = 0; theta < 2 * PI; theta += dtheta) {
      costmap_2d::MapLocation p;
      double x = radius_ * cos(theta) + a_point.point.x;
      double y = radius_ * sin(theta) + a_point.point.y;
      map.worldToMap(x, y, p.x, p.y);
      polygon.push_back(p);
    //  ROS_INFO("Point[%f]: (%i, %i)", theta, p.x, p.y);
    }

    ROS_INFO("outlined circle with %lu cells", polygon.size());

    // get all cells within the cirle
    ROS_INFO("filling circle");
    std::vector<costmap_2d::MapLocation> polygon_cells;
    map.convexFillCells(polygon, polygon_cells);
    ROS_INFO("Filled circle with %lu cells", polygon_cells.size());


    // score each cell and find the max
    ROS_INFO("scoring");

    // prepare the occupancy grid
    grid_.info.resolution = map.getResolution();
    grid_.header.stamp = ros::Time::now();
    grid_.info.origin.position.x = a_point.point.x - map.getResolution() - radius_;
    grid_.info.origin.position.y = a_point.point.y - map.getResolution() - radius_;
    grid_.info.origin.position.z = 0.0;
    grid_.info.origin.orientation.w = 1.0;
    grid_.info.width = 2*(map.cellDistance(radius_));
    grid_.info.height = 2*(map.cellDistance(radius_));
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

    // loop through every cell in polygon and select the highest scoring one
    for(int i = 0; i != polygon_cells.size(); i++) {


      const int x = polygon_cells[i].x, y = polygon_cells[i].y;
      // don't score the attention point itself!
      if (x == a_point_grid_x && y == a_point_grid_y) continue;

      // find the score
      double map_cost = map.getCost(x, y), score;

      if(map_cost == 255) {
        score = -1;
      } else {
        score = 100 - map.getCost(x, y);
      }


      //ROS_INFO("Score1: %f", score);

/*
      if(score != -1) {
        const double a_point_dist = sqrt(pow((double)x - a_point_grid_x, 2) +
                                         pow((double)y - a_point_grid_y, 2));
        score = 0.7 * score + 0.3 * (1.0/(a_point_dist / map.cellDistance(radius_)));
      }
      */


      ROS_INFO("Score: %f", score);


      // store the score in the occupancy grid for visualization
      int index = (y - y_offset + map.cellDistance(radius_)) * grid_.info.width + (x - x_offset + map.cellDistance(radius_));
      if(index > 0 and index < grid_.info.width * grid_.info.height) {
        grid_.data[index] = score;
      }



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
    map.mapToWorld(max_loc.x, max_loc.y, pose.x, pose.y);
    pose.theta = 0;
    ROS_INFO("Returning...");

    return pose;
  }
}
