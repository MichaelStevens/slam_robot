#ifndef FRONTIERFINDER_H_
#define FRONTIERFINDER_H_
#include <nav2d_navigator/GridMap.h>

typedef std::vector<unsigned int> Frontier;
int findFrontiers(GridMap* map, unsigned int start, std::vector<Frontier> &frontiers,
                          ros::Publisher* publisher = NULL, double minTargetAreaSize = 10.0);

#endif // FRONTIERFINDER_H_
