/*
 * test gtsam graph slam in offline mode
 *
 * */

#include <sstream>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include "camera_node.h"
#include "g2o_graph.h"
#include "g2o_parameter.h"
#include "matching_result.h"

using namespace std;

void test_g2o_with_vo_result(); 

string g_vo_file(""); 

int main(int argc, char* argv[])
{
  if(argc < 2){
    cout<<"usage: ./test_g2o [vo_result_file] "<<endl; 
    return 0; 
  }

  g_vo_file = argv[1]; 

  test_g2o_with_vo_result(); 

  return 0; 
}

bool read_vo_file(vector<MatchingResult>& rs); 

void test_g2o_with_vo_result()
{
  vector<MatchingResult> rs;
  if(!read_vo_file(rs)) return; 

  // set graph strcuture 
  GraphG2O g2o_graph; 
  // g2o_graph.mp_optimizer->setVerbose(true); 
  CamNode * pn = new CamNode(); 
  g2o_graph.firstNode(pn); 

  for(int i=0; i<rs.size(); i++){
    MatchingResult& mr = rs[i]; 
    if(mr.edge.id1 != mr.edge.id2 -1){
      // if(mr.inlier_points < 10) continue; 
    }

    g2o_graph.addToGraph(mr, false); 
    if(mr.edge.id2 >= g2o_graph.m_graph_map.size()){
      pn = new CamNode(g2o_graph.m_graph_map.size());
      g2o_graph.m_graph_map[pn->m_id] = pn;  
    }
  }

  // before optimization 
  g2o_graph.writeG2O("test_before_op.g2o");
  g2o_graph.optimizeGraph(100); 

  // after optimization 
  g2o_graph.writeG2O("test_after_op.g2o");

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