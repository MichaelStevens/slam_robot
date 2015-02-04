#ifndef GOAL_GENERATOR_ROS_H
#define GOAL_GENERATOR_ROS_H

#include <nav_msgs/OccupancyGrid.h>
#include <exploration/GenerateGoal.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d.h>
#include <exploration/goal_generator.h>


namespace exploration {
  /*
  * A wrapper class that interfaces GoalGenerator with ROS
  */
  class GoalGeneratorROS {
  public:
    GoalGeneratorROS();
    ~GoalGeneratorROS() {
      delete costmap_;
      delete tf_listener_;
    }

  //private:
    // stores the most recent costmap, converted from an occupancy grid
    costmap_2d::Costmap2D* costmap_;
    // for publishing a grid of scores, for visualization
    ros::Publisher score_pub_;
    tf::TransformListener* tf_listener_;
    GoalGenerator goalGenerator_;
    ros::Subscriber costmap_sub_;
    ros::ServiceServer service_;

    costmap_2d::Costmap2D* occupancyGrid2CostMap(const nav_msgs::OccupancyGrid::ConstPtr &grid);

    bool handleGenerateGoal(exploration::GenerateGoal::Request &req,
                            exploration::GenerateGoal::Response &res);

    void updateOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr &msg);
  };
}

#endif
