/*
 * Oct. 17, 2016, David Z 
 * 
 * Graph interface with g2o
 *
 * */

#ifndef G2O_GRAPH_H
#define G2O_GRAPH_H

#include <map>
// #include <tf/tf.h>
#include "color.h"
#include "camera_node.h"

namespace g2o{ 
  class SparseOptimizer; 
}

class MatchingResult;

extern g2o::SparseOptimizer* create_g2o_instance(); 

class GraphG2O 
{

public:
  GraphG2O(); 
  virtual ~GraphG2O();
  
  g2o::SparseOptimizer* createOptimizer(); 

  bool addToGraph(MatchingResult&, bool set_estimate);
  double error();                             // graph->error();
  size_t camnodeSize();                       // number of camera nodes 
  void writeG2O(std::string ouf);             // save trajectory into [g2o].txt Pose2 or Pose3   

  int m_seq_id;                          // for each new node, m_sequence_id ++ 

  std::map<int, CamNode*> m_graph_map;    // associate node with its m_id 
  g2o::SparseOptimizer * mp_optimizer;  // point to the factor graph

  void firstNode(CamNode* n);
  void optimizeGraph(int iter = 20); 
};

#endif
