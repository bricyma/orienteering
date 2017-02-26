#ifndef PATHPLANNING_H
#define PATHPLANNING_H

#include <vector>
#include "utils/topology.h"


std::vector<std::vector<unsigned int>> incrementalMultiPath(const unsigned int &num_robots, const std::vector<node_t> &waypoints, const unsigned int &rounds);

#endif /* PATHPLANNING_H */

