#include "PathPlanning.h"

#include "assignment/Hungarian.h"
#include "assignment/BipartiteGraph.h"

std::vector<std::vector<unsigned int>> matchingsToPath(const unsigned int &num_robots, const std::vector<EID> &matchings, const std::vector<unsigned int> &prev_matchings_indices, const std::vector<unsigned int> &remaining_indices) {
  std::vector<std::vector<unsigned int>> final_paths(num_robots);

  for (unsigned int i = 0; i < num_robots; i++) {
    std::vector<unsigned int> matching_path;
    unsigned int mathching_index = matchings[matchings.size() - num_robots + i].second;
    while (true) {
//      std::cout << "mathching_index " << mathching_index << std::endl;
      if (mathching_index >= matchings.size() - num_robots) {
        matching_path.push_back(remaining_indices[mathching_index - prev_matchings_indices.size()]);
//        std::cout << "index " << " " << remaining_indices[mathching_index - prev_matchings_indices.size()] << std::endl;
      } else {
        matching_path.push_back(prev_matchings_indices[mathching_index]);
//        std::cout << "index " << " " << prev_matchings_indices[mathching_index] << std::endl;
      }
      
      if (mathching_index >= matchings.size() - num_robots) { 
        break;
      }
      mathching_index = matchings[mathching_index].second;
    }
    
    // ggggggggggggg
    for (unsigned int j = 0; j < matching_path.size(); j++) {
      matching_path[j] += num_robots;
    }
    // ggggggggggggg
    
    final_paths[i] = matching_path;
  }
  
  return final_paths;
}

void pathScoreAndLength(const std::vector<node_t> &input_waypoints, const std::vector<unsigned int> &path_index, int &score, double &length) {
  point2d_t difference = input_waypoints[path_index[0]].pos - input_waypoints[0].pos;
  length += std::sqrt(std::pow(difference.x(), 2) + std::pow(difference.y(), 2));
  
  score += input_waypoints[0].score;
      
  for (unsigned int i = 0; i < path_index.size(); i++) {
    score += input_waypoints[path_index[i]].score;
    if (i < path_index.size() - 1) {
      point2d_t difference = input_waypoints[path_index[i]].pos - input_waypoints[path_index[i + 1]].pos;
      length += std::sqrt(std::pow(difference.x(), 2) + std::pow(difference.y(), 2));
    }
  }
}

std::vector<std::vector<unsigned int>> incrementalMultiPath(const unsigned int &num_robots, const std::vector<node_t> &input_waypoints, const unsigned int &rounds) {
//  std::cout << "input_waypoints.size() " << input_waypoints.size() << std::endl;

  const double max_length = 999999;
//  std::cout << "start_id " << start_id << std::endl;
//  std::cout << "input_waypoints[start_id].pos.x() " << input_waypoints[start_id].pos.x() << std::endl;
//  std::cout << "input_waypoints[start_id].pos.y() " << input_waypoints[start_id].pos.y() << std::endl;

  std::vector<node_t> waypoints = input_waypoints;
  std::vector<node_t> starting_points;
  for (unsigned int i = 0; i < num_robots; i++) {
    starting_points.push_back(input_waypoints[i]);
    waypoints.erase(waypoints.begin());
  }
//  std::cout << "waypoints.size() " << waypoints.size() << std::endl;
  
  // Initialization
  std::vector<unsigned int> prev_matchings_indices;
  std::vector<unsigned int> remaining_indices(waypoints.size());
  for (unsigned int i = 0; i < waypoints.size(); i++) {
    remaining_indices[i] = i;
  }
  std::vector<EID> final_matchings;
    
  std::vector<std::vector<unsigned int>> final_paths;
  int besti = 0;
  int bestScore = 0;
  double bestLength = 0;
  
  // Utility matrix
  for (unsigned int i = 0; i < rounds; i++) {
    std::cout << "i " << i << std::endl;
    Matrix utility_matrix(prev_matchings_indices.size() + num_robots); // rows
    for (unsigned int j = 0; j < utility_matrix.size(); j++) {
      utility_matrix[j].resize(waypoints.size()); // columns

      double x1 = 0;
      double y1 = 0;
      if (j < prev_matchings_indices.size()) { // previous matching points
        x1 = waypoints[prev_matchings_indices[j]].pos.x();
        y1 = waypoints[prev_matchings_indices[j]].pos.y();
      } else { // remaining waypoints
        x1 = starting_points[j - prev_matchings_indices.size()].pos.x();
        y1 = starting_points[j - prev_matchings_indices.size()].pos.y();
      }

      for (unsigned int k = 0; k < prev_matchings_indices.size(); k++) {
        double x2 = waypoints[prev_matchings_indices[k]].pos.x();
        double y2 = waypoints[prev_matchings_indices[k]].pos.y();
        double distance = std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
        utility_matrix[j][k].SetWeight(-distance);
      }

      for (unsigned int k = 0; k < remaining_indices.size(); k++) {
        double x2 = waypoints[remaining_indices[k]].pos.x();
        double y2 = waypoints[remaining_indices[k]].pos.y();
        double distance = std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
        utility_matrix[j][prev_matchings_indices.size() + k].SetWeight(-distance);
      }
    }

    // Parameterize on the diagonal of previous matchings
    for (unsigned int j = 0; j < prev_matchings_indices.size(); j++) {
//      std::cout << "waypoints[prev_matchings_indices[j]].id " << waypoints[prev_matchings_indices[j]].id << std::endl;
//      std::cout << "waypoints[prev_matchings_indices[j]].score " << waypoints[prev_matchings_indices[j]].score << std::endl;
      double lambda = waypoints[prev_matchings_indices[j]].score / 100;
      if (lambda <= 0.2) {
        lambda = 0;
      }
//      const double lambda = std::log10(waypoints[prev_matchings_indices[j]].score) / 2;
//      const double lambda = 1.0;
//      const double lambda = 0.99;
//      std::cout << lambda << std::endl;
      double min = max_length;
      utility_matrix[j][j].SetWeight(max_length);
      for (unsigned int k = 0; k < utility_matrix[j].size(); k++) {
        min = std::min(utility_matrix[j][k].GetWeight(), min);
      }
      utility_matrix[j][j].SetWeight(min * lambda);
    }
    
//    for (unsigned int j = 0; j < utility_matrix.size(); j++) {
//      for (unsigned int k = 0; k < utility_matrix[j].size(); k++) {
//        std::cout << utility_matrix[j][k].GetWeight() << " ";
//      }
//      std::cout << std::endl;
//    }
//    std::cout << "-----------------" << std::endl;
    
    //define a bipartite graph
    BipartiteGraph bg(utility_matrix);

    //run Hungarian methodu
    Hungarian h(bg);
    std::vector<EID> matchings(utility_matrix.size());
    h.HungarianAlgo(matchings);
//    h.DisplayConfig(bg);
    
    std::vector<std::vector<unsigned int>> temp_paths = matchingsToPath(num_robots, matchings, prev_matchings_indices, remaining_indices);
    int score = 0;
    double length = 0;
    for (unsigned int j = 0; j < temp_paths.size(); j++) {
      pathScoreAndLength(input_waypoints, temp_paths[j], score, length);
    }

    if (final_paths.size() == 0 || score > bestScore) {
      final_paths = temp_paths;
      besti = i;
      bestScore = score;
      bestLength = length;
      std::cout << "score " << score << std::endl;
    }
    
    if (i == rounds -1) {
        final_matchings = matchings;
        break;
    }
    
    std::vector<unsigned int> temp_matchings_indices;
    temp_matchings_indices.reserve(utility_matrix.size());
    for (const auto &p: matchings) {
//      std::cout << "p " << " " << p.second;
      if (p.second < prev_matchings_indices.size()) {
        temp_matchings_indices.push_back(prev_matchings_indices[p.second]);
//        cout << " index " << prev_matchings_indices[p.second] << std::endl;
      } else {
        temp_matchings_indices.push_back(remaining_indices[p.second - prev_matchings_indices.size()]);
//        cout << " index " << remaining_indices[p.second - prev_matchings_indices.size()] << std::endl;
      }
    }
    prev_matchings_indices = temp_matchings_indices;
    remaining_indices.clear();
    remaining_indices.reserve(waypoints.size() - prev_matchings_indices.size());
    for (unsigned int j = 0; j < waypoints.size(); j++) {
      bool remain = true;
      for (unsigned int k = 0; k < prev_matchings_indices.size(); k++) {
        if (prev_matchings_indices[k] == j) {
          remain = false;
          break;
        }
      }
      if (remain) {
        remaining_indices.push_back(j);
      }
    }

//    std::cout << "prev_matchings_indices.size() " << prev_matchings_indices.size() << std::endl;
//    for (const auto &index : prev_matchings_indices) {
//      std::cout << "index " << index << " point  " << waypoints[index].pos.x() << " " << waypoints[index].pos.y() << std::endl;
//    }
//
//    std::cout << "remaining_indices.size() " << remaining_indices.size() << std::endl;
//    for (const auto &index : remaining_indices) {
//      std::cout << "index " << index << " point  " << waypoints[index].pos.x() << " " << waypoints[index].pos.y() << std::endl;
//    }
  }

  std::cout << "besti " << besti << std::endl;
  for (unsigned int i = 0; i < final_paths.size(); i++) {
    std::cout << "final_paths[i].size() " << final_paths[i].size() << std::endl;
  }
  
  return final_paths;
}