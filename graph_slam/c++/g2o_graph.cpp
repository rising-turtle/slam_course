

#include "g2o_graph.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/config.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/factory.h"
#include "g2o/types/slam3d/se3quat.h"
#include "g2o/types/slam3d/edge_se3.h"

#include <fstream>
#include <utility>
#include <iostream>
#include "matching_result.h"
#include "g2o_parameter.h"

// using namespace g2o; 
using namespace std;
using namespace CG;
using namespace g2o; 

typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,3> > SlamBlockSolver; 
typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearCSparseSolver; 
// typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearCholmodSolver; 
// typedef std::tr1::unordered_map<int, g2o::HyperGraph::Vertex*> VertexIDMap; 
 typedef std::pair<int, g2o::HyperGraph::Vertex*> VertexIDPair; 
 typedef std::set<g2o::HyperGraph::Edge*> EdgeSet; 


g2o::SparseOptimizer* create_g2o_instance()
{
  g2o::SparseOptimizer * mp_optimizer = new g2o::SparseOptimizer(); 
  mp_optimizer->setVerbose(true); 
  
  // SlamLinearCSparseSolver * linearSolver = new SlamLinearCSparseSolver(); 
  // SlamBlockSolver* solver = new SlamBlockSolver(linearSolver); 
  // g2o::OptimizationAlgorithmLevenberg * algo = new g2o::OptimizationAlgorithmLevenberg(solver); 
  // mp_optimizer->setAlgorithm(algo); 
  std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
  linearSolver = g2o::make_unique<g2o::LinearSolverCSparse<g2o::BlockSolver_6_3::PoseMatrixType>>();
  // g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
     // g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));

  g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(
      g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));

  mp_optimizer->setAlgorithm(solver); 
  return mp_optimizer;
}

GraphG2O::GraphG2O() : 
mp_optimizer(NULL)
{
  createOptimizer(); 
}
GraphG2O::~GraphG2O()
{
  for(map<int, CamNode*>::iterator it = m_graph_map.begin(); 
      it != m_graph_map.end(); ++it)
  {
    delete it->second; 
  }
  m_graph_map.clear();

  if(mp_optimizer)
  {
    mp_optimizer->clear(); 
    delete mp_optimizer; 
    mp_optimizer = 0; 
  }
}


g2o::SparseOptimizer* GraphG2O::createOptimizer()
{
  if(mp_optimizer != NULL) delete mp_optimizer; 
  mp_optimizer = create_g2o_instance();

  return mp_optimizer; 
}


void GraphG2O::firstNode(CamNode* n)
{
  // 1, ids 
  n->m_id = m_graph_map.size(); 
  m_seq_id = 0; 
  n->m_seq_id = ++ m_seq_id;

  // 2, first Pose and add into graph 
  g2o::VertexSE3* reference_pose = new g2o::VertexSE3; 
  reference_pose->setId(n->m_id); 
  reference_pose->setFixed(true);
  mp_optimizer->addVertex(reference_pose); 
  m_graph_map[n->m_id] = n;
  return ;
}

bool GraphG2O::addToGraph(MatchingResult& mr, bool set_estimate)
{
  g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(mp_optimizer->vertex(mr.edge.id1)); 
  g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(mp_optimizer->vertex(mr.edge.id2)); 
  
  if(!v1 && !v2)
  {
    // ROS_ERROR("%s two nodes %i and %i both not exist!", __FILE__, mr.edge.id1, mr.edge.id2); 
    return false;
  }else if(!v1)
  {
    // ROS_WARN("this case is weired, has not solved it"); 
    v1 = new g2o::VertexSE3; 
    int v_id = mr.edge.id1; 
    v1->setId(v_id); 
    v1->setEstimate(v2->estimate() * mr.edge.transform.inverse()); 
    mp_optimizer->addVertex(v1);
  }else if(!v2)
  {
    v2 = new g2o::VertexSE3; 
    int v_id = mr.edge.id2; 
    v2->setId(v_id); 
    v2->setEstimate(v1->estimate() * mr.edge.transform); 
    mp_optimizer->addVertex(v2); 
  }else if(set_estimate){ 
    // set estimate 
    v2->setEstimate(v1->estimate() * mr.edge.transform);
  }

  g2o::EdgeSE3* g2o_edge = new g2o::EdgeSE3; 
  g2o_edge->vertices()[0] = v1; 
  g2o_edge->vertices()[1] = v2; 
  Eigen::Isometry3d meancopy(mr.edge.transform); 
  g2o_edge->setMeasurement(meancopy);
  // g2o_edge->setRobustKernal(&m_robust_kernel);
  g2o_edge->setInformation(mr.edge.informationMatrix); 
  mp_optimizer->addEdge(g2o_edge); 
  return true; 
}

void GraphG2O::optimizeGraph(int iter)
{
  // TODO: parameterize
  // int iter = 20; 
  int currIt = 0; 
  mp_optimizer->initializeOptimization(); 
  double chi2 = 0; 
  double pre_chi2 = 0; 
  for(int i=0; i<iter; i+=currIt)
  {
    currIt = mp_optimizer->optimize(ceil(iter/10));
    chi2 = error(); 
    if(i >= 1){
      if(fabs(pre_chi2 - chi2) <= 1e-7){
        break; 
      } 
    }
    pre_chi2 = chi2; 
  }
  return ;
}

double GraphG2O::error()
{
  mp_optimizer->computeActiveErrors();
  return mp_optimizer->chi2(); 
}


size_t GraphG2O::camnodeSize()
{
  return m_graph_map.size(); 
}

void GraphG2O::writeG2O(std::string f)
{
  ofstream ouf(f); 
  mp_optimizer->save(ouf); 
}



