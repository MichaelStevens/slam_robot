#include <exploration/goal_generator_ros.h>
#include "ros/ros.h"

using namespace std;
using namespace exploration;

namespace exploration {

  GoalGeneratorROS::GoalGeneratorROS() : goalGenerator_(3) {
    // initialize subscriptions, adverisements, and services
    ros::NodeHandle n;

    // to retrieve costmap data
    n.subscribe("/move_base/global_costmap/costmap", 1,
                &GoalGeneratorROS::updateOccupancyGrid, this);

    // to provide ros interface with GoalGenerator
    n.advertiseService("generate_goal",
                       &GoalGeneratorROS::handleGenerateGoal, this);

    // to visualize the goal generation algorithm
    score_pub_ = n.advertise<nav_msgs::OccupancyGrid>("goal_score_map", 1000);

    // tf
    tf_listener_ = new tf::TransformListener(ros::Duration(10));
  }


  bool GoalGeneratorROS::handleGenerateGoal(exploration::GenerateGoal::Request &req,
                                            exploration::GenerateGoal::Response &res) {
    ROS_INFO("Recieved Generate Goal Request!");
    // can't do anything if these are null!
    if(costmap_ == NULL || tf_listener_ == NULL || score_pub_ == NULL) return false;

    // get robot position to pass to goal generator
    tf::StampedTransform transform;
    try {
      tf_listener_->lookupTransform("/map", "/base_link", ros::Time(0), transform);
    } catch(tf::TransformException ex) {
      ROS_INFO("Could not find transform!");
      return false;
    }
    // get the goal
    res.goal_pose = goalGenerator_.generateGoal(req.attention_point, transform.getOrigin().x(),
                                               transform.getOrigin().y(), *costmap_);
    // visualize the algorithm
    score_pub_.publish(*goalGenerator_.getOccupancyGrid());
    return true;
  }

  void GoalGeneratorROS::updateOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    // TODO subscribe the occupancy grid updates instead so you don't have to create a new
    //      object every time
    delete costmap_;
    costmap_ = occupancyGrid2CostMap(msg);
  }

  costmap_2d::Costmap2D* GoalGeneratorROS::occupancyGrid2CostMap(const nav_msgs::OccupancyGrid::ConstPtr &grid) {


    costmap_2d::Costmap2D* costmap = new costmap_2d::Costmap2D(grid->info.width, grid->info.height,
                                                               grid->info.resolution, grid->info.origin.position.x,
                                                               grid->info.origin.position.y);

    ROS_INFO("Constructed costmap of size (%i, %i)", grid->info.width, grid->info.height);

    // copy all the values from the occupancy grid to the costmap
    for(int x = 0; x < costmap_->getSizeInCellsX(); x++) {
      for(int y = 0; y < costmap_->getSizeInCellsY(); y++) {
        int i = costmap_->getIndex(x, y);
        costmap_->setCost(x, y, grid->data[i]);
      }
    }
    ROS_INFO("Copied %i values", costmap_->getSizeInCellsX() * costmap_->getSizeInCellsY());
    return costmap_;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "exploration");
  exploration::GoalGeneratorROS goal_gen;

  ros::spin();
  return 0;
}
