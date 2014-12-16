#include "ros/ros.h"
#include "Frontier.h"
#include <visualization_msgs/Marker.h>

typedef std::multimap<double,unsigned int> Queue;
typedef std::pair<double,unsigned int> Entry;

//	publisher = navigatorNode.advertise<visualization_msgs::Marker>("frontiers", 1, true);

void findCluster(GridMap* map, double* plan, unsigned int* offset,
																 std::vector<Frontier> &frontiers,unsigned int &frontierCells,
																 unsigned int startCell, double minTargetAreaSize) {
	// Create a new frontier and expand it
	Frontier front;
	int frontNumber = -2 - frontiers.size();
	int minAreaSize = minTargetAreaSize / (map->getResolution() * map->getResolution());

	// Initialize a new queue with the found frontier cell
	Queue frontQueue;
	frontQueue.insert(Entry(0.0, startCell));
	bool isBoundary = false;

	//	Queue unexplQueue;
	//	int areaSize = 0;
	while(!frontQueue.empty()) {
		// Get the nearest cell from the queue
		Queue::iterator next = frontQueue.begin();
		double distance = next->first;
		unsigned int index = next->second;
		unsigned int x, y;
		frontQueue.erase(next);

		// Check if it is a frontier cell
		if(!map->isFrontier(index)) continue;

		// Add it to current frontier
		front.push_back(index);
		frontierCells++;

		// Add all adjacent cells to queue
		for(unsigned int it = 0; it < 4; it++)
		{
			int i = index + offset[it];
			if(map->isFree(i) && plan[i] == -1)
			{
				plan[i] = distance + map->getResolution();
				frontQueue.insert(Entry(distance + map->getResolution(), i));
			}
		}
	}
	frontiers.push_back(front);
}

int findFrontiers(GridMap* map, unsigned int start, std::vector<Frontier> &frontiers,
									ros::Publisher* publisher, double minTargetAreaSize) {
	// Create some workspace for the wavefront algorithm

	unsigned int mapSize = map->getSize();
	double* plan = new double[mapSize];
	for(unsigned int i = 0; i < mapSize; i++)	plan[i] = -1;

	unsigned int offset[8];
	offset[0] = -1;					// left
	offset[1] =  1;					// right
	offset[2] = -map->getWidth();	// up
	offset[3] =  map->getWidth();	// down
	offset[4] = -map->getWidth() - 1;
	offset[5] = -map->getWidth() + 1;
	offset[6] =  map->getWidth() - 1;
	offset[7] =  map->getWidth() + 1;

	// 1. Frontiers identification and clustering
	// =========================================================================
	unsigned int frontierCells = 0;

	// Initialize the queue with the robot position
	Queue queue;
	Entry startPoint(0.0, start);
	queue.insert(startPoint);
	plan[start] = 0;
	Queue::iterator next;
	unsigned int x, y, index;
	double linear = map->getResolution();
	int cellCount = 0;

	// Search for frontiers with wavefront propagation
	while(!queue.empty()) {
		cellCount++;

		// Get the nearest cell from the queue
		next = queue.begin();
		double distance = next->first;
		index = next->second;
		queue.erase(next);

		// Now continue 1st level WPA
		for(unsigned int it = 0; it < 4; it++) {
			unsigned int i = index + offset[it];
		//	ROS_INFO("Value =  %i", map->getData(i));
			if(plan[i] == -1 && map->isFree(i)) {
				// Check if it is a frontier cell
				if(map->isFrontier(i)) {
					//ROS_INFO("Fount Frontier cell");
					findCluster(map, plan, offset, frontiers, frontierCells, i, minTargetAreaSize);
				} else {
					//ROS_INFO("Found cell to contrinue search");
					queue.insert(Entry(distance+linear, i));
				}
				plan[i] = distance+linear;
			}
		}
	}
	delete[] plan;
	ROS_DEBUG("[MinPos] Found %d frontier cells in %d frontiers.", frontierCells, (int)frontiers.size());
	if(frontiers.size() == 0) {
		if(cellCount > 50) {
			//return 0;
		} else	{
			ROS_WARN("[MinPos] No Frontiers found after checking %d cells!", cellCount);
			//return 0;
		}
	}

	// Publish frontiers as marker for RVIZ

	if(true)	{

		visualization_msgs::Marker marker;
		marker.header.frame_id = "/map";
		marker.header.stamp = ros::Time();
		marker.id = 0;
		marker.type = visualization_msgs::Marker::CUBE_LIST;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = map->getOriginX() + (map->getResolution() / 2);
		marker.pose.position.y = map->getOriginY() + (map->getResolution() / 2);
		marker.pose.position.z = map->getResolution() / 2;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = map->getResolution();
		marker.scale.y = map->getResolution();
		marker.scale.z = map->getResolution();
		marker.color.a = 0.5;
		marker.color.r = 1.0;
		marker.color.g = 1.0;
		marker.color.b = 1.0;
		marker.points.resize(frontierCells);
		marker.colors.resize(frontierCells);

		unsigned int p = 0;
		srand(1337);
		for(unsigned int i = 0; i < frontiers.size(); i++) {
			char r = rand() % 256;
			char g = rand() % 256;
			char b = rand() % 256;
			for(unsigned int j = 0; j < frontiers[i].size(); j++)	{
				if(p < frontierCells)	{
					if(!map->getCoordinates(x, y, frontiers[i][j]))	{
						ROS_ERROR("[MinPos] getCoordinates failed!");
						break;
					}
					marker.points[p].x = x * map->getResolution();
					marker.points[p].y = y * map->getResolution();
					marker.points[p].z = 0;

					marker.colors[p].r = r;
					marker.colors[p].g = g;
					marker.colors[p].b = b;
					marker.colors[p].a = 1;
				} else {
					ROS_ERROR("[MinPos] SecurityCheck failed! (Asked for %d / %d)", p, frontierCells);
				}
				p++;
			}
		}
		publisher->publish(marker);
		ROS_INFO("Published Frontiers");
	}

	return 0;
}
