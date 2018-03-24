#include "featureMap/Optimizer.h"

ThetaOptimizer::ThetaOptimizer():
    m_Optimizer(new g2o::SparseOptimizer)
{
    m_LinearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    m_SolverPtr = new Block(m_LinearSolver);
    m_LMSolver = new g2o::OptimizationAlgorithmLevenberg(m_SolverPtr);

    m_Optimizer->setAlgorithm(m_LMSolver);
    m_Optimizer->setVerbose(false);
}

ThetaOptimizer::~ThetaOptimizer()
{
    delete m_Optimizer;
    delete m_SolverPtr;
    delete m_LinearSolver;
}

void ThetaOptimizer::ResetOptimizer()
{
    delete m_Optimizer;
    m_Optimizer = new g2o::SparseOptimizer();
}

void ThetaOptimizer::AddVertex(const float initVal)
{
    m_Vertex.setEstimate((double)initVal);
    m_Vertex.setId(0);
    m_Optimizer->addVertex(&m_Vertex);
}

void ThetaOptimizer::AddEdge(const vector<float> inputs, const cv::Mat measurent, const int id)
{
    ThetaOptimizationEdge edge = ThetaOptimizationEdge(inputs);
    edge.setId(id);
    edge.setVertex(0, &m_Vertex);
    edge.setMeasurement(measurent);
    //edge.setInformation(Eigen::Matrix<double,3,4>::Identity());
    m_Optimizer->addEdge(&edge);
}

float ThetaOptimizer::StartOptimize(const int times)
{
    m_Optimizer->initializeOptimization();
    m_Optimizer->optimize(times);
    return m_Vertex.estimate();
}

float ThetaOptimizer::GetOptimizeOutput() const
{
    return m_Vertex.estimate();
}

void ThetaOptimizer::SetVerboseTrue()
{
    m_Optimizer->setVerbose(true);
}

void ThetaOptimizer::SetVerboseFalse()
{
    m_Optimizer->setVerbose(false);
}
