
/*
 * Oct. 3, 2016, David Z 
 * 
 * Graph interface with gtsam 
 *
 * */

#include "gtsam_graph.h"

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
#include <fstream>
#include <iostream>
// #include <QList>
// #include <QThread>
// #include <QtConcurrentMap>
// #include <qtconcurrentrun.h>
#include "matching_result.h"
#include "camera_node.h"
#include "gt_parameter.h"
#include "chi2.h"
// #include "transformation_estimation_euclidean.h"

using namespace gtsam;
using namespace std; 

#define INVALID_NUM 111

using symbol_shorthand::X; // Pose3 (x, y, z, r, p, y)
using symbol_shorthand::V; // Vel (xdot, ydot, zdot)
using symbol_shorthand::B; // Bias (ax, ay, az, gx, gy, gz) 
using symbol_shorthand::L; // Plane landmark (nv, d)
using symbol_shorthand::Q; // Point3 (x, y, z)

Eigen::Matrix<double, 6, 1> cov_Helper(Eigen::Matrix4f& m)
{
  Eigen::Matrix4d md = m.cast<double>(); 
  Pose3 p(md); 
  gtsam::Vector6 r = Pose3::ChartAtOrigin::Local(p); 
  return r; 
} 

ofstream* CGraphGT::getRecFile()
{
  static ofstream * pf = 0 ;
  if(pf == 0)
  {
    // pf = new ofstream("vro_results.log"); 
    pf = new ofstream(CGTParams::Instance()->m_vro_result.c_str());
  }
  return pf; 
}

CGraphGT::CGraphGT()
{
  mp_fac_graph = new NonlinearFactorGraph; 
  mp_new_fac = new NonlinearFactorGraph; 
  mp_node_values = new Values ;
  mp_new_node = new Values; 

  initISAM2Params(); 
  mp_w2o = new Pose3;
  mp_u2c = new Pose3; 
  mp_prev_bias = new imuBias::ConstantBias; 
  mp_prev_state = new NavState;
  // mb_record_vro_results = true; // TODO: parameterize this one  
  mb_record_vro_results = CGTParams::Instance()->m_record_vro_results; 
  // m_plane_landmark_id = 0; 
  // m_sift_landmark_id =0 ;
}

void CGraphGT::initISAM2Params()
{
  mp_isam2_param = new ISAM2Params; 
  mp_isam2_param->relinearizeThreshold = 0.1; // 0.3 0.2
  mp_isam2_param->relinearizeSkip = 1; // 2
  mp_isam2 = new ISAM2(*mp_isam2_param); 
}

CGraphGT::~CGraphGT(){
  
  if(mp_prev_bias != NULL) 
  {
    delete mp_prev_bias; mp_prev_bias = NULL;
  }
  if(mp_prev_state != NULL)
  {
    delete mp_prev_state; mp_prev_state = NULL; 
  }

  if(mp_fac_graph != NULL) 
  {
    delete mp_fac_graph;  mp_fac_graph = NULL; 
  }
  
  if(mp_new_fac != NULL) 
  {
    delete mp_new_fac; mp_new_fac = NULL; 
  }

  if(mp_new_node != NULL)
  {
    delete mp_new_node; mp_new_node = NULL; 
  }

  if(mp_node_values != NULL)
  {
    delete mp_node_values; mp_node_values = NULL; 
  }

  if(mp_w2o != NULL)
  {
    delete mp_w2o;  mp_w2o = NULL; 
  }
  
  if(mp_u2c != NULL)
  {
    delete mp_u2c;  mp_u2c = NULL; 
  }

  if(mp_isam2_param != NULL)
  {
    delete mp_isam2_param; mp_isam2_param = NULL; 
  }

  if(mp_isam2 != NULL)
  {
    delete mp_isam2; mp_isam2 = NULL; 
  }

  for(map<int, CamNode*>::iterator it = m_graph_map.begin(); 
      it != m_graph_map.end(); ++it)
  {
    delete it->second; 
  }

}

void CGraphGT::writeGTSAM(string f)
{
  ofstream ouf(f); 
  if(!ouf.is_open())
  {
    cerr<<__FILE__<<": failed to open file "<<f<<" to writeGTSAM structure"<<endl; 
    return ; 
  }
  
  mp_fac_graph->saveGraph(ouf, *mp_node_values); 
  
}

double CGraphGT::error()
{
  return mp_fac_graph->error(*mp_node_values);
}

void CGraphGT::setWorld2Original(double p)
{
  //// WORLD Coordinate System 
  //          
  //          Z   X
  //          |  /
  //          | /
  //     Y____|/
  //        
  //   CAMERA Coordinate System    
  //           Z 
  //          /
  //         / 
  //        /  
  //        ------- X
  //        |
  //        |
  //        |Y
  //
  //
  Rot3 R_g2b = Rot3::RzRyRx(-M_PI/2., 0, -M_PI/2.); // roll, pitch, yaw, in WORLD coordinate system
  Rot3 R_b2o = Rot3::RzRyRx(p ,0 , 0); 
  Rot3 R_g2o = R_g2b * R_b2o; 
  // Rot3 R_g2o = R_b2o * R_g2b; 

  Point3 t = Point3::Zero(); 
  (*mp_w2o) = Pose3::Create(R_g2o, t);
   t<< 1, 3, 2; 
   cout << t<< " after rotation "<<endl <<(*mp_w2o)*t<<endl;
   cout << "rotation pitch: "<< R_b2o * t<<endl; 
   cout << "rotation all: "<< R_g2b * R_b2o * t<<endl; 
}

void CGraphGT::setCamera2IMUTranslation(double px, double py, double pz) // only set translation 
{
    Rot3 R_g2o;  
    Point3 t(px, py, pz); 
    (*mp_u2c) = Pose3::Create(R_g2o, t);
}


void CGraphGT::setCamera2IMU(double p)
{
  //// Body/IMU Coordinate System 
  //          
  //             X
  //            /
  //           /
  //          /_____ Y
  //          |
  //          |
  //        Z | 
  //
  //        
  //   CAMERA Coordinate System    
  //           Z 
  //          /
  //         / 
  //        /  
  //        ------- X
  //        |
  //        |
  //        |Y
  //

  Rot3 R_g2b = Rot3::RzRyRx( M_PI/2., 0. , M_PI/2.); // roll, pitch, yaw, in BODY coordinate system 
  Rot3 R_b2o = Rot3::RzRyRx(p ,0 , 0); 
  Rot3 R_g2o = R_g2b * R_b2o; 
  // Rot3 R_g2o = R_b2o * R_g2b; 

  Point3 t = Point3::Zero(); 
  (*mp_u2c) = Pose3::Create(R_g2o, t);
   t<< 1, 3, 2; 
   cout << t<< " after rotation "<<endl <<(*mp_u2c)*t<<endl;
   cout << "rotation pitch: "<< R_b2o * t<<endl; 
   cout << "rotation all: "<< R_g2b * R_b2o * t<<endl; 
}

// input ax, ay, az represents normalized gravity vector 
// at initial phase where the imu is assumed static 
void CGraphGT::initFromImu(double ax, double ay, double az)
{
    if(m_graph_map.size() <= 0)
    {
	   cerr<<"No node has been created before calling initFromImu!"<<endl; 
	   return ; 
    }
    
    // compute rotation for the first pose 
    Eigen::Vector3d fv(ax, ay, az); 
    Eigen::Vector3d tv(0, 0, 1);  // vn100's gz points to upwards
    Eigen::Vector3d w = fv.cross(tv).normalized(); 
    double angle = acos(fv.dot(tv)); 
    
    double half_angle = angle /2.;
    Eigen::Vector4d vq; 
    vq.head<3>() = w * sin(half_angle); 
    vq[3] = cos(half_angle); 
    Eigen::Quaterniond q(vq); 
    Eigen::Matrix<double, 3, 3> m = q.toRotationMatrix(); 
    
    // cout <<"fv= "<<endl<<fv<<endl;
    // cout <<"m = "<<endl<<m<<endl; 
    
    Eigen::Vector3d dst_t = m * fv; 
    // cout <<"dst_t: "<<dst_t.normalized()<<endl; 

    Rot3 R(m);
    Point3 t(0, 0, 0); 
    Pose3 new_pose(R,t); 
    
    *(mp_w2o) = new_pose; 

    // new_pose.print("new_pose");
    // mp_node_values->update(X(m_graph_map[0]->m_id), new_pose); 
    // mp_new_node->update(X(m_graph_map[0]->m_id), new_pose); 
    return ; 
}

void CGraphGT::firstNode(CamNode* n, bool online)
{
    // 1, ids 
    n->m_id = m_graph_map.size();
    m_sequence_id = 0; 
    if(online)
    {
      n->m_seq_id = ++ m_sequence_id; 
    }

    // 2, first Pose and add into graph 
    Pose3 origin_priorMean(Eigen::Matrix4d::Identity());
    mp_node_values->insert(X(n->m_id), origin_priorMean);
    // for isam2
    mp_new_node->insert(X(n->m_id), origin_priorMean);

    Vector6 s; // s << 1e-5, 1e-5, 1e-5, 1e-3, 1e-3, 1e-3;
    s << 1e-7, 1e-7, 1e-7, 1e-7, 1e-7, 1e-7;
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(s);
    mp_fac_graph->add(PriorFactor<Pose3>(X(n->m_id), origin_priorMean, priorNoise));

    // for isam2
    mp_new_fac->add(PriorFactor<Pose3>(X(n->m_id), origin_priorMean, priorNoise));

    m_graph_map[n->m_id] = n; 

    // 3, imu part 
    Vector3 priorVelocity; priorVelocity << 0, 0, 0; 
    mp_node_values->insert(V(n->m_id), priorVelocity); 
    mp_new_node->insert(V(n->m_id), priorVelocity); 

    imuBias::ConstantBias priorBias; 
    mp_node_values->insert(B(n->m_id), priorBias); 
    mp_new_node->insert(B(n->m_id), priorBias); 
    
    // Assemble prior noise model and add it in the graph 
    // noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6)<< 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished()); // rad, rad, rad, m, m, m
    noiseModel::Diagonal::shared_ptr velocity_noise_model = noiseModel::Isotropic::Sigma(3, 1e-3); // m/s 
    noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);  

   mp_fac_graph->add(PriorFactor<Vector3>(V(n->m_id), priorVelocity, velocity_noise_model)); 
   mp_fac_graph->add(PriorFactor<imuBias::ConstantBias>(B(n->m_id), priorBias, bias_noise_model));
   
   // for isam2 
   mp_new_fac->add(PriorFactor<Vector3>(V(n->m_id), priorVelocity, velocity_noise_model)); 
   mp_new_fac->add(PriorFactor<imuBias::ConstantBias>(B(n->m_id), priorBias, bias_noise_model));
}

bool CGraphGT::addToGTSAM(gtsam::NavState& new_state, int vid, bool add_pose )
{
  if(add_pose)
  {
    mp_node_values->insert(X(vid), new_state.pose());
    mp_new_node->insert(X(vid), new_state.pose());
  }
  // ROS_INFO("Insert V(%d) and B(%d)", vid, vid);
  mp_node_values->insert(V(vid), new_state.v()); 
  mp_node_values->insert(B(vid), *mp_prev_bias); 
  mp_new_node->insert(V(vid), new_state.v()); 
  mp_new_node->insert(B(vid), *mp_prev_bias); 

  return true; 
}

bool CGraphGT::addToGTSAM(MatchingResult& mr, bool set_estimate)
{
  bool pre_exist = mp_node_values->exists(X(mr.edge.id1)); 
  bool cur_exist = mp_node_values->exists(X(mr.edge.id2));
  Pose3 inc_pose(mr.edge.transform.matrix()); 
  // inc_pose.print("inc_pose has value");
  // 

  // This is the bug, detected at Jan. 19, 2017, David Z
  // inc_pose = (*mp_u2c).inverse()*inc_pose*(*mp_u2c); 
  inc_pose = (*mp_u2c)*inc_pose*(*mp_u2c).inverse(); 

  if(!pre_exist && !cur_exist)
  {
    // ROS_ERROR("%s two nodes %i and %i both not exist ", __FILE__, mr.edge.id1, mr.edge.id2); 
    return false; 
  }
  else if(!pre_exist)
  {
    // ROS_WARN("this case is weired, has not solved it!");
    Pose3 cur_pose = mp_node_values->at<Pose3>(X(mr.edge.id2)); 
    Pose3 pre_pose = cur_pose * inc_pose.inverse(); 
    mp_node_values->insert(X(mr.edge.id1), pre_pose); 
    mp_new_node->insert(X(mr.edge.id1), pre_pose); 
  }
  else if(!cur_exist)
  {
    Pose3 pre_pose = mp_node_values->at<Pose3>(X(mr.edge.id1)); 
    Pose3 cur_pose = pre_pose * inc_pose;
    mp_node_values->insert(X(mr.edge.id2), cur_pose);
    mp_new_node->insert(X(mr.edge.id2), cur_pose); 
  }else if(set_estimate)
  {  
    // set estimate 
    // ROS_WARN("%s should not arrive here %i to set estimate", __FILE__, __LINE__);
    Pose3 pre_pose = mp_node_values->at<Pose3>(X(mr.edge.id1)); 
    Pose3 cur_pose = pre_pose * inc_pose; 
    mp_node_values->update(X(mr.edge.id2), cur_pose);
    mp_new_node->update(X(mr.edge.id2), cur_pose); 
  }

  // fake odo matrix 
  Eigen::Matrix<double, 6, 6> tmp = Eigen::Matrix<double, 6, 6>::Identity()*1000000; 
  // noiseModel::Gaussian::shared_ptr visual_odometry_noise = noiseModel::Gaussian::Information(tmp);

  Eigen::Matrix<double, 6, 6> Adj_Tuc = (*mp_u2c).AdjointMap(); 
  tmp = Adj_Tuc * mr.edge.informationMatrix * Adj_Tuc.transpose(); 

  // Eigen::Matrix<double, 6, 6> I6 = Eigen::Matrix<double, 6, 6>::Identity(); 
  // Eigen::Matrix<double, 6, 6> cov = tmp.inverse(); 
  // cov = cov * tmp; 
  // if(!MatrixEqual(cov, I6, 1e-5))
  {
    // cout <<"what between id1: "<<mr.edge.id1<<" and "<<mr.edge.id2<<endl
    //  <<" Inf = "<<endl<<tmp<<endl
    //  <<" cov= "<<tmp.inverse()<<endl;
  }

  // noiseModel::Gaussian::shared_ptr visual_odometry_noise = noiseModel::Gaussian::Information(mr.edge.informationMatrix);
  noiseModel::Gaussian::shared_ptr visual_odometry_noise = noiseModel::Gaussian::Information(tmp);

  mp_fac_graph->add(BetweenFactor<Pose3>(X(mr.edge.id1), X(mr.edge.id2), inc_pose, visual_odometry_noise)); 
  mp_new_fac->add(BetweenFactor<Pose3>(X(mr.edge.id1), X(mr.edge.id2), inc_pose, visual_odometry_noise)); 
  
  return true; 
}

void CGraphGT::fakeOdoNode(CamNode* new_node)
{
   // assume id has been set 
   if( new_node->m_id != m_graph_map.size()) 
   { 
     cerr <<__FILE__<<" "<<__LINE__<<" Here this should not happen!" <<endl; 
     new_node->m_id = m_graph_map.size(); 
     new_node->m_seq_id = ++m_sequence_id; 
   }
   
   CamNode* pre_node = m_graph_map[new_node->m_id-1]; 
   MatchingResult mr; 
   mr.edge.id1 = pre_node->m_id; 
   mr.edge.id2 = new_node->m_id; 
   mr.edge.transform.setIdentity();  
   mr.edge.informationMatrix = Eigen::Matrix<double, 6, 6>::Identity()*1e4; 
   addToGTSAM(mr, false); 
   
   m_graph_map[new_node->m_id] = new_node; 

   if(mb_record_vro_results)
   {
      recordVROResult(mr);  // record the result of VRO 
   }
   return ;
}

void CGraphGT::readVRORecord(std::string fname)
{
  return readVRORecord(fname, mv_vro_res); 
}

void CGraphGT::readVRORecord(std::string fname, std::vector<MatchingResult*> & mv)
{
  char buf[4096] = {0}; 
  ifstream inf(fname.c_str()); 
  int id_to, id_from; 
  double x,y,z,roll,pitch,yaw; 

  if(!inf.is_open())
  {
    cerr <<" failed to open file "<<fname<<endl;
    return ;
  }

  while(!inf.eof())
  {
    // Pose3 
    gtsam::Vector6 r; 
    MatchingResult* pm = new MatchingResult; 
    // inf>>id_to>>id_from>>x>>y>>z>>roll>>pitch>>yaw; 
    inf>>id_to>>id_from; 
    for(int i=0; i<6; i++)
      inf>>r(i); 
    gtsam::Pose3 p = gtsam::Pose3::ChartAtOrigin::Retract(r); 
    Eigen::Matrix4d md = p.matrix();   
    pm->final_trafo = md.cast<float>();
    pm->edge.transform.matrix() = md; 

    for(int i=0; i<6; i++)
      for(int j=i; j<6; j++)
      {
        inf>>pm->edge.informationMatrix(i,j); 
        pm->edge.informationMatrix(j,i) = pm->edge.informationMatrix(i,j);
      }
    pm->edge.id2 = id_to; 
    pm->edge.id1 = id_from; 
    
    // ROS_INFO("succeed to read record from %d to %d", id_from, id_to); 
    // if(pm->edge.informationMatrix(0,0) == 0) break; 
    if(inf.eof()) break; // For the last line in the file, after >> information(i,j), some whitespace remains which makes inf.eof() = false, and then add one wrong record
    mv.push_back(pm); 
    //if(mv.size() >= 5963)
    { 
      // cout <<" read vro record "<<id_to<<" "<<id_from<<" "<<r<<" information: "<<pm->edge.informationMatrix<<endl;
    }
  }

  cout <<__LINE__<<" read vro records "<<mv.size()<<endl;
  return ; 
}

void CGraphGT::printVROResult(ostream& ouf, MatchingResult& m)
{
  Eigen::Matrix<double, 6, 1> p = cov_Helper(m.final_trafo); 
  ouf<<m.edge.id2<<" "<<m.edge.id1<<" "<<p(0)<<" "<<p(1)<<" "<<p(2) <<" "<<p(3)<<" "<<p(4)<<" "<<p(5)<< " "; 
  for(int i=0; i<6; i++)
    for(int j=i;j<6;j++)
    {
      ouf<<m.edge.informationMatrix(i,j)<<" ";
    }
  ouf<<endl;
  ouf.flush();
  return;
}

void CGraphGT::recordVROResult(MatchingResult& m)  // record vro result 
{
  CamNode* pNow = m_graph_map[m.edge.id2];
  CamNode* pOld = m_graph_map[m.edge.id1]; 
  Eigen::Matrix<double, 6, 1> p = cov_Helper(m.final_trafo); 
  ofstream* pf = getRecFile(); 

  (*pf) << pNow->m_seq_id<<" "<<pOld->m_seq_id<<" "<<p(0)<<" "<<p(1)<<" "<<p(2) <<" "<<p(3)<<" "<<p(4)<<" "<<p(5)<< " "; 
  for(int i=0; i<6; i++)
    for(int j=i;j<6;j++)
    {
      (*pf)<<m.edge.informationMatrix(i,j)<<" ";
    }
  (*pf)<<endl;
  (*pf).flush();
  return ; 
}

// this function is to add new node in offline mode, which means the pose estimation has been known 
bool CGraphGT::addNodeOffline(CamNode* new_node, MatchingResult* mr, bool only_vo)
{
  bool ret = true;
  // it cannot be the first node  
  new_node->m_id = m_graph_map.size(); 
  new_node->m_seq_id = mr->edge.id2;
  CamNode* pre_node = m_graph_map[new_node->m_id-1]; 
  if(only_vo || mr->edge.informationMatrix(0,0) != 10000) // valid match
  {
    m_graph_map[new_node->m_id] = new_node; 
      
    // save previous 
    int pre_id1 = mr->edge.id1; 
    int pre_id2 = mr->edge.id2; 

    // ROS_INFO("Line %d before correction mr->id1 %d mr->id2 %d", __LINE__, mr->edge.id1, mr->edge.id2);
    correctMatchingID(mr); 
    // ROS_INFO("Line %d after correction mr->id1 %d mr->id2 %d", __LINE__, mr->edge.id1, mr->edge.id2);
    // printVROResult(std::cout, *mr);
    addToGTSAM(*mr, true); 

    // recover 
    mr->edge.id1 = pre_id1; 
    mr->edge.id2 = pre_id2;

  }else{ // not a valid match, skip this one 
    // 
    ret = false;
  }
  return ret;
}

// from sequence id to matching id 
void CGraphGT::correctMatchingID(MatchingResult* mr)
{
  int from_id = mr->edge.id1; 
  int to_id = mr->edge.id2; 
  map<int, CamNode*>::iterator it = m_graph_map.begin(); 
  bool from_good = false; 
  bool to_good = false; 
  while(it != m_graph_map.end())
  {
    if(it->second->m_seq_id == from_id)
    {
      mr->edge.id1 = it->second->m_id; 
      from_good = true;
    }
    if(it->second->m_seq_id == to_id) 
    {
      mr->edge.id2 = it->second->m_id; 
      to_good = true; 
    }
    if(from_good && to_good) break;
    ++ it; 
  }
  return ;
}

// add edge in offline mode, which means node has been added, and pose estimation has been known 
void CGraphGT::addEdgeOffline(MatchingResult* mr)
{
  if(mr->edge.informationMatrix(0,0) != 10000)
  {
    // save previous 
    int pre_id1 = mr->edge.id1; 
    int pre_id2 = mr->edge.id2; 

    correctMatchingID(mr); 
    addToGTSAM(*mr, false); 

    // recover 
    mr->edge.id1 = pre_id1; 
    mr->edge.id2 = pre_id2; 
  }
  return ;
}
/*
ADD_RET CGraphGT::addNode(CamNode* new_node)
{
  // first node
  if(m_graph_map.size() == 0)
  {
    firstNode(new_node);
    return SUCC_KF; 
  }
  
  // 
  size_t old_node_size = mp_node_values->size(); 

  // match with previous node 
  new_node->m_id = m_graph_map.size(); 
  new_node->m_seq_id = ++m_sequence_id;
  CamNode* pre_node = m_graph_map[new_node->m_id-1]; 
  MatchingResult mr = new_node->matchNodePair(pre_node); 
  
  int current_best_match = 0; 
  if(mr.succeed_match) // found a trafo
  {
    if(isSmallTrafo(mr))// mr.edge.transform))  //  too small transform, e.g. < 3 degree or < 0.04 meters
    {
      // TODO: 
      // ROS_WARN("new_node %d is too close to its previous node, do not add into graph", new_node->m_seq_id);
      return FAIL_NOT_KF; 
    }else if(!isLargeTrafo(mr))// found a good match 
    {
       //computeCovVRO(new_node, mr);  // compute the actual covariance 
       if(mr.edge.informationMatrix(0, 0) == mr.edge.informationMatrix(0,0))
       {
         addToGTSAM(mr, true);
         current_best_match = mr.inlier_matches.size(); 
         m_graph_map[new_node->m_id] = new_node; 
         if(mb_record_vro_results)
         {
           recordVROResult(mr);  // 
         }
       }
    }
  } else // failed_match 
  {
      ROS_ERROR("%s Found no transformation to predecessor", __FILE__);
  }

  // local loop closures 
  int seq_cand = CGTParams::Instance()->m_lookback_nodes; 
  QList<CamNode* > nodes_to_comp;   // nodes to compare 
  if(m_graph_map.size() > 3)
  {
    int n_id = new_node->m_id - 2;
    for(int j=0; j<seq_cand && n_id >= 0 ; j++ )
    {
      nodes_to_comp.push_back(m_graph_map[n_id--]); 
    }
  } 
  
  // Concurrently work on comparison 
  if(nodes_to_comp.size() > 0)
  {
    QThreadPool *qtp = QThreadPool::globalInstance(); 
    QList<MatchingResult> results = QtConcurrent::blockingMapped(nodes_to_comp, boost::bind(&CamNode::matchNodePair, new_node, _1)); 
    for(int i=0; i<results.size(); i++)
    {
      MatchingResult& mr = results[i]; 
      if(mr.succeed_match)
      {
        if(!isSmallTrafo(mr) && !isLargeTrafo(mr)) // not a too small and large transformation 
        {
          bool reset_estimate = mr.inlier_matches.size() > current_best_match; 
          if(reset_estimate) current_best_match = mr.inlier_matches.size(); 
          computeCovVRO(new_node, mr);     // compute the actual covariance 
          if(mr.edge.informationMatrix(0,0) != mr.edge.informationMatrix(0,0)) // nan information matrix 
            continue; 
          addToGTSAM(mr, reset_estimate);  // add factor and values 
          m_graph_map[new_node->m_id] = new_node;  // add into graph map
          if(mb_record_vro_results)
          {
            recordVROResult(mr); 
          }
        }
      }
    }
  }
  {//TODO: NonConcurrently 
  }
  
  // End of the function 
  bool b_found_trafo = this->mp_node_values->size() > old_node_size; 
  
  // TODO: keep the unmatched node? 
  if(b_found_trafo) 
    return SUCC_KF; 

  // return b_found_trafo;
  return FAIL_KF; 
}*/

void CGraphGT::optimizeGraphIncremental()
{
  mp_isam2->update(*mp_new_fac, *mp_new_node); 
  // mp_isam2->update(); 
  (*mp_node_values) = mp_isam2->calculateEstimate(); 
  // clear graph and nodes 
  mp_new_fac->resize(0); 
  mp_new_node->clear(); 
}


void CGraphGT::optimizeGraph()
{
  return CGraphGT::optimizeGraphBatch();
}

void CGraphGT::optimizeGraphBatch()
{
   mp_fac_graph->print("Factor graph:\n"); 
   // Values initial_value = (*mp_node_values); 
   double ini_error = mp_fac_graph->error(*mp_node_values); 
   
   LevenbergMarquardtOptimizer optimizer(*mp_fac_graph, *mp_node_values); 
   (*mp_node_values) = optimizer.optimize(); 
   mp_node_values->print("Final results\n"); 
   cout <<" initial errors: "<< ini_error <<endl; 
   cout <<" final errors: "<<mp_fac_graph->error(*mp_node_values)<<endl;
}

bool CGraphGT::isSmallTrafo(MatchingResult& mr)
{
  Eigen::Isometry3d& T = mr.edge.transform; 
  double dist = T.translation().norm(); 
  if(dist > CGTParams::Instance()->m_small_translation) return false; 

  double angle = acos((T.rotation().trace()-1)*0.5) * 180./M_PI; 
  if(angle > CGTParams::Instance()->m_small_rotation) return false; 

  return true;
}

bool CGraphGT::isLargeTrafo(MatchingResult& mr)
{
  Eigen::Isometry3d& T = mr.edge.transform; 
  double dist = T.translation().norm(); 
  if(dist > CGTParams::Instance()->m_large_translation) return true; 

  double angle = acos((T.rotation().trace()-1)*0.5) * 180./M_PI; 
  if(angle > CGTParams::Instance()->m_large_rotation) return true; 

  return false; 
}

size_t CGraphGT::camnodeSize()
{
  return m_graph_map.size(); 
}

bool CGraphGT::writeTrajectory(std::string f)
{
  ofstream ouf(f.c_str()); 
  if(!ouf.is_open())
  {
    printf("%s failed to open f: %s to write trajectory!\n", __FILE__, f.c_str()); 
    return false; 
  }
  
  for(map<int, CamNode*>::iterator it = m_graph_map.begin(); 
      it != m_graph_map.end(); ++it)
  {
    Pose3 p = mp_node_values->at<Pose3>(X(it->first)); 
    p = (*mp_w2o)*p;
    // Vector3 rpy = p.rotation().rpy();
    // ouf<<it->first<<" "<<p.x()<<" "<<p.y()<<" "<<p.z()<<" "<<rpy(0)<<" "<<rpy(1)<<" "<<rpy(2)<<" "<<it->second->m_seq_id<<endl;
    // Quaternion 
    gtsam::Quaternion q = p.rotation().toQuaternion(); 
    ouf<<it->first<<" "<<p.x()<<" "<<p.y()<<" "<<p.z()<<" "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<" "<<it->second->m_seq_id<<endl;
  }
  return true; 
}

bool CGraphGT::trajectoryPLY(std::string f, CG::COLOR c)
{
  ofstream ouf(f.c_str()); 
  if(!ouf.is_open())
  {
    printf("%s %d failed to open f: %s to write trajectory!\n", __FILE__,__LINE__, f.c_str()); 
    return false; 
  }
  
  // first, add header 
  int vertex_number = m_graph_map.size(); 
  headerPLY(ouf, vertex_number);

  for(map<int, CamNode*>::iterator it = m_graph_map.begin(); 
      it != m_graph_map.end(); ++it)
  {
    Pose3 p = mp_node_values->at<Pose3>(X(it->first)); 
    p = (*mp_w2o)*p;
    ouf<<p.x()<<" "<<p.y()<<" "<<p.z()<<" "<<(int)CG::g_color[c][0]<<" "<<(int)CG::g_color[c][1]<<" "<<(int)CG::g_color[c][2]<<endl;
  }
  ouf.close(); 
  return true; 
}
/*
bool CGraphGT::mapPLY(std::string f, std::string img_dir, int skip, float depth_scale, CSparseFeatureVO* pSF)
{
  // generate a global point cloud 
  ofstream ouf(f.c_str()); 
  if(!ouf.is_open())
  {
    printf("%s %d failed to open f: %s to write map!\n", __FILE__,__LINE__, f.c_str()); 
    return false; 
  }
  
  vector<float> pts_loc; 
  vector<unsigned char> pts_col;
  cv::Mat i_img, d_img; 
  CSReadCV r4k; 

  for(map<int, CamNode*>::iterator it = m_graph_map.begin(); 
      it != m_graph_map.end(); ++it)
  {
    Pose3 p = mp_node_values->at<Pose3>(X(it->first)); 
    p = (*mp_w2o)*p;
 
    // traverse all the points in this node 
    // get image 
    stringstream ss; 
    ss<<img_dir<<"/"<<setfill('0')<<setw(7)<<it->second->m_sequence_id<<".bdat";  // setw(4)
    r4k.readOneFrameCV(ss.str(), i_img, d_img);

    // compute local point cloud 
    vector<float> p_loc; 
    vector<unsigned char> p_col; 
    pSF->generatePointCloud(i_img, d_img, skip, depth_scale, p_loc, p_col); 

    // transform into global 
    Point3 fo, to; 
    for(int i=0; i<p_loc.size(); i+=3)
    {
      fo << p_loc[i], p_loc[i+1], p_loc[i+2]; 
      to = p.transform_from(fo); 
      p_loc[i] = to(0); p_loc[i+1] = to(1); p_loc[i+2] = to(2); 
    }
    
    pts_loc.insert(pts_loc.end(), p_loc.begin(), p_loc.end()); 
    pts_col.insert(pts_col.end(), p_col.begin(), p_col.end());

    // ouf<<p.x()<<" "<<p.y()<<" "<<p.z()<<" "<<g_color[c][0]<<" "<<g_color[c][1]<<" "<<g_color[c][2]<<endl;
  }
  
  // output into file 

  // first, add header 
  int vertex_number = pts_loc.size()/3; 
  headerPLY(ouf, vertex_number);

  for(int i=0; i<pts_loc.size(); i+=3)
  {
    ouf<< pts_loc[i]<<" "<<pts_loc[i+1]<<" "<<pts_loc[i+2]<<" "<<pts_col[i]<<" "<<pts_col[i+1]<<" "<<pts_col[i+2]<<endl;
  }
  ouf.close(); 
  return true;
}*/

void CGraphGT::headerPLY(std::ofstream& ouf, int vertex_number)
{
  ouf << "ply"<<endl
    <<"format ascii 1.0"<<endl
    <<"element vertex "<<vertex_number<<endl
    <<"property float x"<<endl
    <<"property float y"<<endl
    <<"property float z"<<endl
    <<"property uchar red"<<endl
    <<"property uchar green"<<endl
    <<"property uchar blue"<<endl
    <<"end_header"<<endl;
}

void CGraphGT::writeG2O(std::string f)
{
  gtsam::writeG2o(*mp_fac_graph, *mp_node_values, f); 
  return ;
}
