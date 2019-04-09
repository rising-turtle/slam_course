;/*
	Apr. 8 2019, He Zhang, hzhang8@vcu.edu 

	gtsam interface 

*/

#include <sstream>
#include <string>
#include <vector>
#include <fstream>
#include "camera_node.h"
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/OrientedPlane3Factor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/inference/Symbol.h>
#include "gtsam_graph.h"
#include "matching_result.h"

using namespace std;
using namespace CG; 
using namespace gtsam; 

void test_gtsam_with_vo_result(); 

string g_vo_file(""); 

int main(int argc, char* argv[])
{
  if(argc < 2){
    cout<<"usage: ./test_gtsam vo_result_file "<<endl; 
    return 0; 
  }

  g_vo_file = argv[1]; 

  test_gtsam_with_vo_result(); 

  return 0; 
}

bool read_vo_file(vector<MatchingResult>& rs); 

void test_gtsam_with_vo_result()
{
  vector<MatchingResult> rs;
  if(!read_vo_file(rs)) return; 

  // set graph strcuture 
  CGraphGT gt_graph; 
   
  // g2o_graph.mp_optimizer->setVerbose(true); 
  CamNode * pn = new CamNode(); 
  gt_graph.firstNode(pn); 

  for(int i=0; i<rs.size(); i++){
    MatchingResult& mr = rs[i]; 

    gt_graph.addToGTSAM(mr, false); 
    if(mr.edge.id2 >= gt_graph.m_graph_map.size()){
      pn = new CamNode(gt_graph.m_graph_map.size());
      gt_graph.m_graph_map[pn->m_id] = pn;  
    }
  }

  // before optimization 
  gt_graph.trajectoryPLY("test_before_gtsam.log", BLUE);
  gt_graph.optimizeGraph(); 

  // after optimization 
  gt_graph.trajectoryPLY("test_after_gtsam.log", RED);

  return ;
}


bool read_vo_file(vector<MatchingResult>& rs)
{
  ifstream inf(g_vo_file.c_str()); 
  if(!inf.is_open()){
    return false; 
  }

  rs.clear(); 
  while(!inf.eof()){
    string s;
    getline(inf,s);
    if(!s.empty())
    {
      stringstream ss; 
      ss << s; 
      MatchingResult m; 
      float x, y, z, qx, qy, qz, qw; 
      int inliers; 
      ss >> m.edge.id2>>m.edge.id1>>x>>y>>z>>qx>>qy>>qz>>qw; 
      for(int i=0; i<6; i++)
        for(int j = i; j<6; j++){
            ss>>m.edge.informationMatrix(i,j); 
            // cout <<" m.edge.informationMatrix(i,j) = "<<m.edge.informationMatrix(i,j)<<endl; 
            // if(j!=i)
            m.edge.informationMatrix(j,i) = m.edge.informationMatrix(i,j);
        }

      Eigen::Matrix4f T = Eigen::Matrix4f::Identity(); 
      T(0,3) = x; T(1,3) = y; T(2,3) = z; 
      Eigen::Quaternionf q(qw, qx, qy, qz);
      // m.inlier_points = inliers;
      T.block<3,3>(0,0) = q.normalized().toRotationMatrix(); 
      m.final_trafo = T;  m.ransac_trafo = T; 
      m.edge.transform.matrix() = T.cast<double>(); 
      // m.edge.informationMatrix = Eigen::Matrix<double, 6, 6>::Identity() * inliers;
      // if(inf.eof()) break; 
      rs.push_back(m); 
    }
  }
  return true;

}