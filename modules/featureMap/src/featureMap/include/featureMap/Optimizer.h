#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <Eigen/Core>
#include <cmath>
#include <chrono>
#include <opencv2/opencv.hpp>
#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"

using namespace std;

typedef g2o::BlockSolver<g2o::BlockSolverTraits<1, 1> > Block;

class ThetaOptimizationVertex: public g2o::BaseVertex<1, double>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void setToOriginImpl()
    {
        _estimate = 0;
    }

    virtual void oplusImpl(const double* update)
    {
        _estimate += *update;
    }

    virtual bool read(istream &is) {return true;}
    virtual bool write(ostream &os) const {return true;}
};

class ThetaOptimizationEdge: public g2o::BaseUnaryEdge<12, cv::Mat, ThetaOptimizationVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ThetaOptimizationEdge(vector<float> inputs)
    {
        if (inputs.size() == 6)
        {
            _x = (cv::Mat_<float> (3,1) << inputs[0], inputs[1], inputs[2]);
            _l = (cv::Mat_<float> (3,1) << inputs[3], inputs[4], inputs[5]);
        }
    }

    void computeError()
    {
        if (_measurement.cols == 4 && _measurement.rows == 3)
        {
            const ThetaOptimizationVertex* thetaV = static_cast<const ThetaOptimizationVertex*>(_vertices[0]);
            const float theta = thetaV->estimate();

            float c = cos(theta / 180.0 * M_PI);
            float s = sin(theta / 180.0 * M_PI);
            float ux = _l.at<float>(0,0);
            float uy = _l.at<float>(1,0);
            float uz = _l.at<float>(2,0);
            float ux2 = ux * ux;
            float uy2 = uy * uy;
            float uz2 = uz * uz;
            cv::Mat Rtheta = (cv::Mat_<float> (3,3) <<
                              c+ux2*(1-c), ux*uy*(1-c)-uz*s, ux*uy*(1-c)+uy*s,
                              uy*ux*(1-c)+uz*s, c+uy2*(1-c), uy*uz*(1-c)-ux*s,
                              uz*ux*(1-c)-uy*s, uz*uy*(1-c)+ux*s, c+uz2*(1-c));

            cv::Mat _q = _measurement.colRange(0,1);
            cv::Mat _cov = _measurement.colRange(1,4);
            cv::Mat _covInv = _cov.inv();
            cv::Mat score = -(((Rtheta*_x-_q).t()) * _covInv * (Rtheta*_x-_q));
            _error(0,0) = 0 - std::exp(score.at<float>(0,0) / 2);
        }
    }

    virtual bool read(istream &is) {return true;}
    virtual bool write(ostream &os) const {return true;}

    cv::Mat _x;
    cv::Mat _l;
};

class ThetaOptimizer
{
public:
    ThetaOptimizer();
    ~ThetaOptimizer();

    void ResetOptimizer();

    void AddVertex(const float initVal);

    void AddEdge(const vector<float> inputs, const cv::Mat measurent, const int id);

    float StartOptimize(const int times);

    float GetOptimizeOutput() const;

    void SetVerboseTrue();

    void SetVerboseFalse();

private:
    g2o::SparseOptimizer* m_Optimizer;
    Block::LinearSolverType* m_LinearSolver;
    Block* m_SolverPtr;
    g2o::OptimizationAlgorithmLevenberg* m_LMSolver;

    ThetaOptimizationVertex m_Vertex;
};

#endif //OPTIMIZER_H
