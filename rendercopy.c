  std::vector<std::vector<unsigned int>> paths = incrementalMultiPath(1, nodes, nodes.size() / 2 - 1);
  std::cout << "paths.size() " << paths.size() << std::endl;
  
  for (unsigned int i = 0; i < paths.size(); i++) {
    paths[i].insert(paths[i].begin(), i);
    order2.push_back(paths[i]);
  }

  int total_score = 0;
  double total_length = 0;
  std::vector<double> lengths(paths.size());
  for (unsigned int i = 0; i < order2.size(); i++) {
    endnodes.push_back(order2[i][order2[i].size() - 1]);
    double length = 0;
    for (unsigned int j = 0; j < order2[i].size(); j++) {
      std::cout << order2[i][j] << " ";
      total_score += nodes[order2[i][j]].score;
      if (j < order2[i].size() - 1) {
        point2d_t difference = nodes[order2[i][j]].pos - nodes[order2[i][j + 1]].pos;
        length += std::sqrt(std::pow(difference.x(), 2) + std::pow(difference.y(), 2));
      }
    }
    lengths[i] = length;
    std::cout << "length " << length << std::endl;
    total_length += length;
  }
  std::cout << std::endl;
  std::cout << "total score: " << total_score << std::endl;
  std::cout << "total length: " << total_length << std::endl;
  std::cout << "**********average " << total_score / total_length << std::endl;

  //  double t_max = max_length;  //define t_max as the same as above algorithm's cost
  double a = 0.5, b = 0.2, c = 0.3;
  OP.setStartEnd(StartID, endnodes[0]);
  OP.init();
  OP.constructRoute(lengths[0], a, b, c);
  OP.improveRoute(lengths[0], a, b, c);
  OP.gravityStepIteration(lengths[0], a, b, c);
  order.push_back(OP.path());
  double op_cost1 = OP.totalCost();

  /*int StartID2 = 1;
  OP.setStartEnd(StartID2, endnodes[1]);
  OP.init();
  OP.constructRoute(lengths[1], a, b, c);
  OP.improveRoute(lengths[1], a, b, c);
  OP.gravityStepIteration(lengths[1], a, b, c);
  order.push_back(OP.path());
  double op_cost2 = OP.totalCost();*/

  int score_op = 0;
  bool flag_order[node_num];
  for (unsigned int i = 0; i < node_num; i++) {
    flag_order[i] = false;
  }
  for (unsigned int i = 0; i < order[0].size(); i++) {
    if (!flag_order[order[0][i]]) {
      score_op += nodes[order[0][i]].score;
      flag_order[order[0][i]] = true;
    }
  }
/*  for (unsigned int i = 0; i < order[1].size(); i++) {
    if (!flag_order[order[1][i]]) {
      score_op += nodes[order[1][i]].score;
      flag_order[order[1][i]] = true;
    }
  }*/
  cout << "total score: " << score_op << endl;
  cout << "total cost: " << op_cost1 << endl;
  cout << "*************average: " << score_op / (op_cost1 ) << endl;
