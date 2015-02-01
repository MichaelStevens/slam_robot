#ifndef GOAL_GENERATOR_H
#define GOAL_GENERATOR_H

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PointStamped.h>
#include <costmap_2d/costmap_2d.h>
#include <cmath>


namespace exploration {
  class GoalGenerator {
  public:
    GoalGenerator(const int radius);
    
    geometry_msgs::Pose2D generateGoal(geometry_msgs::PointStamped &attention_point,
                                       double robot_x, double robot_y,
                                       costmap_2d::Costmap2D& grid);

    nav_msgs::OccupancyGrid* getOccupancyGrid() {
      return &grid_;
    }
  private:
    const int radius_;
    costmap_2d::Costmap2D* occupancyGrid2CostMap(nav_msgs::OccupancyGrid::ConstPtr &grid);
    nav_msgs::OccupancyGrid grid_;





  };
}

#endif
