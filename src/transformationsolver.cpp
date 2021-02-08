#include "transformationsolver.h"
#include <ceres/ceres.h>
#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <math.h>
#include "feature.h"
#include "matching.h"

Eigen::Matrix3d euler2rotm(double theta, double phi, double psi)
{
    Eigen::Matrix3d rotx;
    rotx << 1,0,0,
            0,cos(theta),-sin(theta),
            0,sin(theta),cos(theta);
    Eigen::Matrix3d roty;
    roty << cos(phi),0,sin(phi),
            0,1,0,
            -sin(phi),0,cos(phi);
    Eigen::Matrix3d rotz;
    rotz << cos(psi), -sin(psi),0,
            sin(psi),cos(psi),0,
            0,0,1;
    return rotx*roty*rotz;
}

class EdgeCostFunctor {
public:
    EdgeCostFunctor(Edge edge1, Edge edge2): edge1_(edge1), edge2_(edge2){}

    bool operator()(const double* x, double* error) const {
        Eigen::Vector3d T;
        T << x[0],x[1],x[2];
        Eigen::Matrix3d R;
        R = euler2rotm(x[3],x[4],x[5]);

        Eigen::Vector3d bary1 = edge1_.getBarycenter().cast<double> ();
        Eigen::Vector3d bary2 = edge2_.getBarycenter().cast<double> ();
        Eigen::Matrix3d C;
        C << 0.5, 0, 0,
                0, 0.5, 0,
                0, 0, 1000;
        Eigen::Vector3d d = bary2-R*(bary1-T);
        error[0] = d.transpose()*(C + R*C*R.transpose()).inverse()*d;
        return true;
    }
private:
    Edge edge1_;
    Edge edge2_;
};

class EdgeCostFunctor2d {
public:
    EdgeCostFunctor2d(Edge edge1, Edge edge2): edge1_(edge1), edge2_(edge2){}

    bool operator()(const double* x, double* error) const {
        Eigen::Vector2d T;
        T << x[0], x[1];
        Eigen::Matrix2d R;
        R << cos(x[5]), -sin(x[5]),
                sin(x[5]), cos(x[5]);
        Eigen::Vector2d bary1 = edge1_.getBarycenter().head(2).cast<double> ();
        Eigen::Vector2d bary2 = edge2_.getBarycenter().head(2).cast<double> ();
        error[0] = (bary2-R*(bary1-T)).norm();
        return true;
    }
private:
    Edge edge1_;
    Edge edge2_;
};

class PlaneCostFunctor {
public:
    PlaneCostFunctor(Plane plane1, Plane plane2):plane1_(plane1), plane2_(plane2){}

    bool operator()(const double* x, double* error) const {
        Eigen::Matrix3d R;
        R = euler2rotm(x[3],x[4],x[5]);

        Eigen::Vector3d normal1 = plane1_.getDirection().cast<double> ();
        Eigen::Vector3d normal2 = plane2_.getDirection().cast<double> ();
        error[0] = std::abs((R*normal1).dot(normal2)-normal1.norm()*normal2.norm());
        return true;
    }
private:
    Plane plane1_;
    Plane plane2_;
};


TransformationSolver::TransformationSolver()
{
}

Eigen::VectorXd TransformationSolver::getRigidTransformation(Eigen::MatrixXi corespEdge, Eigen::MatrixXi corespPlane,
                                                             std::vector<Edge> edgeVect1, std::vector<Edge> edgeVect2,
                                                             std::vector<Plane> planeVect1, std::vector<Plane> planeVect2,
                                                             std::vector<double> *residuals, double *cost)
{
    // build the problem
    ceres::Problem problem;
    ceres::CostFunction* cost_functionEdge;
    ceres::CostFunction* cost_functionPlane;
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    std::vector<ceres::ResidualBlockId> psrIDs;


    // initialize the transformation
    double x[6] = {0.0,0.0,0.0,0.0,0.0,0.0};

    // add the edge residuals
    for (int i=0; i<corespEdge.rows(); i++){
        cost_functionEdge =
                new ceres::NumericDiffCostFunction<EdgeCostFunctor2d, ceres::CENTRAL, 1, 6>(new EdgeCostFunctor2d(edgeVect1[corespEdge(i,0)], edgeVect2[corespEdge(i,1)]));
        ceres::ResidualBlockId ID = problem.AddResidualBlock(cost_functionEdge, NULL, x);
        psrIDs.push_back(ID);
    }
    // add the plane residuals
    for (int i=0; i<corespPlane.rows(); i++){
        cost_functionPlane =
                new ceres::NumericDiffCostFunction<PlaneCostFunctor, ceres::CENTRAL, 1, 6>(new PlaneCostFunctor(planeVect1[corespPlane(i,0)], planeVect2[corespPlane(i,1)]));
        problem.AddResidualBlock(cost_functionPlane, NULL, x);
    }

    // here we set the bounds of our problem (parameter, index, bound)
    problem.SetParameterLowerBound(x, 0, -1);
    problem.SetParameterLowerBound(x, 1, -0.01);
    problem.SetParameterLowerBound(x, 2, -1);
    problem.SetParameterLowerBound(x, 3, -M_PI/6);
    problem.SetParameterLowerBound(x, 4, -M_PI/6);
    problem.SetParameterLowerBound(x, 5, -M_PI/6);
    problem.SetParameterUpperBound(x, 0, 2);
    problem.SetParameterUpperBound(x, 1, 0.01);
    problem.SetParameterUpperBound(x, 2, 2);
    problem.SetParameterUpperBound(x, 3, M_PI/6);
    problem.SetParameterUpperBound(x, 4, M_PI/6);
    problem.SetParameterUpperBound(x, 5, M_PI/6);

    // Evaluation options
    ceres::Problem::EvaluateOptions EvalOpts;
    EvalOpts.residual_blocks = psrIDs;

    // run the solver using Levenberg Marquardt algorithm
    options.function_tolerance = 1e-3;
    Solve(options, &problem, &summary);
    problem.Evaluate(EvalOpts, cost, residuals, nullptr, nullptr);

    // build the transformation vector
    Eigen::VectorXd transVec(6);
    transVec << x[0], x[1], 0, x[3], x[4], x[5];
    return transVec;
}

Eigen::VectorXd TransformationSolver::transformationEdge(Eigen::MatrixXi corespEdge,
                                                         std::vector<Edge> edgeVect1, std::vector<Edge> edgeVect2,
                                                         std::vector<double> *residuals, double *cost)
{
    // build the problem
    ceres::Problem problem;
    ceres::CostFunction* cost_functionEdge;
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    std::vector<ceres::ResidualBlockId> psrIDs;


    // initialize the transformation
    double x[6] = {0.0,0.0,0.0,0.0,0.0,0.0};

    // add the edge residuals
    for (int i=0; i<corespEdge.rows(); i++){
        cost_functionEdge =
                new ceres::NumericDiffCostFunction<EdgeCostFunctor2d, ceres::CENTRAL, 1, 6>(new EdgeCostFunctor2d(edgeVect1[corespEdge(i,0)], edgeVect2[corespEdge(i,1)]));
        ceres::ResidualBlockId ID = problem.AddResidualBlock(cost_functionEdge, NULL, x);
        psrIDs.push_back(ID);
    }
    // Evaluation options
    ceres::Problem::EvaluateOptions EvalOpts;
    EvalOpts.residual_blocks = psrIDs;

    // here we set the bounds of our problem (parameter, index, bound)
    problem.SetParameterLowerBound(x, 0, -1);
    problem.SetParameterLowerBound(x, 1, -0.01);
    problem.SetParameterLowerBound(x, 2, -1);
    problem.SetParameterLowerBound(x, 3, -M_PI/6);
    problem.SetParameterLowerBound(x, 4, -M_PI/6);
    problem.SetParameterLowerBound(x, 5, -M_PI/6);
    problem.SetParameterUpperBound(x, 0, 2);
    problem.SetParameterUpperBound(x, 1, 0.01);
    problem.SetParameterUpperBound(x, 2, 2);
    problem.SetParameterUpperBound(x, 3, M_PI/6);
    problem.SetParameterUpperBound(x, 4, M_PI/6);
    problem.SetParameterUpperBound(x, 5, M_PI/6);


    // run the solver using Levenberg Marquardt algorithm
    options.function_tolerance = 1e-3;
    Solve(options, &problem, &summary);
    problem.Evaluate(EvalOpts, cost, residuals, nullptr, nullptr);

    // build the transformation vector
    Eigen::VectorXd transVec(6);
    transVec << x[0], x[1], x[2], x[3], x[4], x[5];
    return transVec;
}

Eigen::VectorXd TransformationSolver::transformationPlane(Eigen::MatrixXi corespPlane,
                                                          std::vector<Plane> planeVect1, std::vector<Plane> planeVect2,
                                                          std::vector<double> *residuals, double *cost)
{
    // build the problem
    ceres::Problem problem;
    ceres::CostFunction* cost_functionPlane;
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    std::vector<ceres::ResidualBlockId> psrIDs;


    // initialize the transformation
    double x[6] = {0.0,0.0,0.0,0.0,0.0,0.0};

    // add the edge residuals
    // add the plane residuals
    for (int i=0; i<corespPlane.rows(); i++){
        cost_functionPlane =
                new ceres::NumericDiffCostFunction<PlaneCostFunctor, ceres::CENTRAL, 1, 6>(new PlaneCostFunctor(planeVect1[corespPlane(i,0)], planeVect2[corespPlane(i,1)]));
        problem.AddResidualBlock(cost_functionPlane, NULL, x);
    }
    // Evaluation options
    ceres::Problem::EvaluateOptions EvalOpts;
    EvalOpts.residual_blocks = psrIDs;

    problem.SetParameterLowerBound(x, 3, -0.05);
    problem.SetParameterLowerBound(x, 4, -0.05);
    problem.SetParameterUpperBound(x, 3, 0.05);
    problem.SetParameterUpperBound(x, 4, 0.05);

    // run the solver using Levenberg Marquardt algorithm
    options.function_tolerance = 1e-3;
    Solve(options, &problem, &summary);
    problem.Evaluate(EvalOpts, cost, residuals, nullptr, nullptr);

    // build the transformation vector
    Eigen::VectorXd transVec(6);
    transVec << x[0], x[1], x[2], x[3], x[4], x[5];
    return transVec;
}

void TransformationSolver::applyTransformation(Eigen::VectorXd transVec, std::shared_ptr<Scan> scan1, std::shared_ptr<Scan> scan2)
{
    Eigen::Matrix3d R;
    Eigen::VectorXd X1 = scan1->getPose();
    Eigen::VectorXd X2 = Eigen::VectorXd::Zero(6);
    X2.tail(3) = X1.tail(3)+ transVec.tail(3);
    R = euler2rotm(X1(3), X1(4), X1(5));
    X2.head(3) = X1.head(3) + R*transVec.head(3);
    scan2->setPose(X2);
}

Scan TransformationSolver::testSolver(Scan scan){
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    // translation along X axis
    transform(0,3) = 0;
    // theta radians around Z axis
    float theta = M_PI/64;
    transform(0,0) = cos(theta);
    transform(0,1) = -sin(theta);
    transform(1,0) = sin(theta);
    transform(1,1) = cos(theta);

    // creation of the transformed scan
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud (*scan.getCloud(), *transformedCloud, transform);
    Scan transformedScan(transformedCloud);

    return transformedScan;

}
