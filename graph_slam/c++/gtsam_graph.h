/*
 * Oct. 3, 2016, David Z 
 * 
 * Graph interface with gtsam 
 *
 * */

#ifndef GTSAM_GRAPH_H
#define GTSAM_GRAPH_H

#include <map>
#include <iostream>
#include <fstream>
#include <vector>
#include <set>
#include <gtsam/base/Matrix.h>
#include "camera_node.h"
#include "color.h"
// #include "opencv2/opencv.hpp"

namespace gtsam{
  class NonlinearFactorGraph;
  class Values;
  class Pose3; 
  class NavState;
  class ISAM2Params; 
  class ISAM2; 
  namespace imuBias
  {
  class ConstantBias;
  }
}

class MatchingResult; 

typedef enum{SUCC_KF, FAIL_NOT_KF, FAIL_KF} ADD_RET; // return status for adding new node: SUCC_KF, succeed to add a kf; FAIL_NOT_KF, succeed, but not a kf, discard it; 
                                                     // FAIL_KF, fail to match.  

class CGraphGT 
{

public:
  CGraphGT(); 
  virtual ~CGraphGT();
  
  void initFromImu(double ax, double ay, double az); // try to reset the pose of the first node, called after firstNode
  
  void firstNode(CamNode*, bool online=true);   // initialize everything 
  // ADD_RET addNode(CamNode*);              // try to add a new node into graph 
  void fakeOdoNode(CamNode*);             // fake a Identity transformation node
  void optimizeGraph();                       // optimize the factor graph 
  void optimizeGraphBatch();
  void optimizeGraphIncremental(); 
 
  bool addToGTSAM(MatchingResult&, bool set_estimate);
  bool addToGTSAM(gtsam::NavState&, int vid, bool add_pose);  // add NavState(Pose, Velocity, Bias into GTSAM)
 
  bool isSmallTrafo(MatchingResult&); 
  bool isLargeTrafo(MatchingResult&); 
  double error();                             // graph->error();
  size_t camnodeSize();                       // number of camera nodes 
  void writeGTSAM(std::string ouf);
  void writeG2O(std::string ouf);             // save trajectory into [g2o].txt Pose2 or Pose3   
  bool writeTrajectory(std::string ouf);      // dump trajectory into [ouf] 

  int m_sequence_id;                          // for each new node, m_sequence_id ++ 
  int m_vertex_id;                    // for each new node in the gtsam, m_vertex_id ++

  std::map<int, CamNode*> m_graph_map;    // associate node with its m_id 
  gtsam::NonlinearFactorGraph* mp_fac_graph;  // point to the factor graph
  gtsam::Values * mp_node_values;             // point to the node values in the factor graph
  void setWorld2Original(double r, double p, double y);   // 
  void setWorld2Original(double p); 
  void setCamera2IMU(double p); 
  void setCamera2IMUTranslation(double px, double py, double pz); // only set translation 
  gtsam::Pose3 * mp_w2o;      // transform from world to original (IMU)
  gtsam::Pose3 * mp_u2c;      // transform from IMU to camera 
  gtsam::imuBias::ConstantBias * mp_prev_bias; 
  gtsam::NavState * mp_prev_state;

  bool mb_record_vro_results;    // whether to record all the estimation from vro
  std::ofstream * getRecFile();       // record results from vro in this file 
  void recordVROResult(MatchingResult& m);  // record vro result 

  void printVROResult(std::ostream& ouf, MatchingResult& m); // printf matching result
  void readVRORecord(std::string inf);   // read vro record and save them into a vector 
  void readVRORecord(std::string inf, std::vector<MatchingResult*> & mv);   // read vro record and save them into a vector 
  
  // offline operation 
  std::vector<MatchingResult*> mv_vro_res;
  bool addNodeOffline(CamNode*, MatchingResult*, bool only_vo = false); 
  void addEdgeOffline(MatchingResult*); 
  // from sequence id to matching id 
  void correctMatchingID(MatchingResult* mr);
  
  // isam2 
  gtsam::ISAM2* mp_isam2; 
  gtsam::ISAM2Params * mp_isam2_param;
  gtsam::NonlinearFactorGraph* mp_new_fac;  // point to the new factor graph
  gtsam::Values * mp_new_node;    // point to the new node values in the factor graph
  void initISAM2Params(); 

  // ply 
  void headerPLY(std::ofstream&, int vertex_number); 
  bool trajectoryPLY(std::string ouf, CG::COLOR);   // output trajectory in ply 
  // bool mapPLY(std::string ouf, std::string img_dir, int skip, float depth_scale, CSparseFeatureVO* pSF);       
};


#endif
