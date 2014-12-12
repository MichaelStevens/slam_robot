#include "ros/ros.h"
#include "FrontierFinder.h"
#include "nav_msgs/GetMap.h"
#include <nav2d_navigator/ExplorationPlanner.h>
#include <iostream>
#include <tf/transform_listener.h>
using namespace std;
int main(int argc, char **argv) {

  ros::init(argc, argv, "exploration");

  ros::NodeHandle n;
  ros::ServiceClient dynamic_map_client = n.serviceClient<nav_msgs::GetMap>("dynamic_map");
  tf::TransformListener tfListener;
  FrontierFinder frontierFinder;
  GridMap* gridMap = new GridMap();

  ros::Rate loop_rate(1);
  while (ros::ok()) {
    // get the current occupancy grid
    nav_msgs::GetMap srv;
    if(dynamic_map_client.call(srv)) {

      gridMap->update(srv.response.map);
      tf::StampedTransform map2base_link;

      try {
        tfListener.lookupTransform("/map", "/base_link", ros::Time(0), map2base_link);
        double res = gridMap->getResolution();
        unsigned int x = (map2base_link.getOrigin().x() - gridMap->getOriginX())
                         / res + 0.5;
        unsigned int y = (map2base_link.getOrigin().y() - gridMap->getOriginY())
                         / res + 0.5;
        unsigned int start;
        if(gridMap->getIndex(x, y, start)) {
          unsigned int _;

          frontierFinder.findExplorationTarget(gridMap, start, _);

          ROS_INFO("Published Frontiers");
        } else {
          ROS_ERROR("Robot location not in occupancy grid!");
        }
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ROS_WARN("Could not obtain transform. Retrying...");
        continue;
      }
    } else {
      ROS_WARN("No response from gmapping. Retrying..");
    }
    ros::spinOnce();

    loop_rate.sleep();

  }

  return 0;
}
