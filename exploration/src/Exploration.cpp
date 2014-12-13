#include "ros/ros.h"
#include "Frontier.h"
#include "nav_msgs/GetMap.h"
#include <nav2d_navigator/GridMap.h>
#include <iostream>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

using namespace std;

int main(int argc, char **argv) {
  ros::init(argc, argv, "exploration");

  ros::NodeHandle n;
  ros::ServiceClient dynamic_map_client = n.serviceClient<nav_msgs::GetMap>("dynamic_map");
  ros::Publisher frontier_publisher = n.advertise<visualization_msgs::Marker>("frontiers", 1, true);
  tf::TransformListener tfListener;
  GridMap* gridMap = new GridMap();

  ros::Rate loop_rate(1);
  while (ros::ok()) {

    // get the current occupancy grid
    nav_msgs::GetMap srv;
    if(!dynamic_map_client.call(srv)) {
      ROS_WARN("No response from gmapping. Retrying..");
      continue;
    }

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
      if(!gridMap->getIndex(x, y, start)) {
        ROS_ERROR("Robot location not in occupancy grid!");
        continue;
      }

      std::vector<Frontier> frontiers;
      findFrontiers(gridMap, start, frontiers, &frontier_publisher);

      ROS_INFO("Published Frontiers");
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ROS_WARN("Could not obtain transform. Retrying...");
      continue;
    }
    ros::spinOnce();

    loop_rate.sleep();

  }

  return 0;
}
