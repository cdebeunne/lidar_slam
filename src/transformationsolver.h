#ifndef TRANSFORMATIONSOLVER_H
#define TRANSFORMATIONSOLVER_H

#include "utility.h"
#include "scan.h"
#include <ceres/ceres.h>
#include <glog/logging.h>

/*!
 * \file transformationsolver.h
 * \brief handles the rigid transformation between two scans
 * \author César Debeunne
 */

class TransformationSolver
{
public:
    TransformationSolver();

    /*!
    *  \brief determine the rigid transformation between two scans thanks to
    * ceres non linear solver using Levenberg-Marquardt  algorithm
    *  \param corespEdge, corespPlane, edgeVect1, edgeVect2, planeVect1, planeVect2,
    * residuals, cost
    *  \return transformation vector
    */
    Eigen::VectorXd getRigidTransformation(Eigen::MatrixXi corespEdge, Eigen::MatrixXi corespPlane,
                                           std::vector<Edge> edgeVect1, std::vector<Edge> edgeVect2,
                                           std::vector<Plane> planeVect1, std::vector<Plane> planeVect2,
                                           std::vector<double> *residuals, double *cost);

    /*!
    *  \brief determine the rigid transformation between two scans with only edge features
    *  \param corespEdge, edgeVect1, edgeVect2, residuals, cost
    *  \return transformation vector
    */
    Eigen::VectorXd transformationEdge(Eigen::MatrixXi corespEdge,
                                       std::vector<Edge> edgeVect1, std::vector<Edge> edgeVect2,
                                       std::vector<double> *residuals, double *cost);

    /*!
    *  \brief determine the rigid transformation between two scans with only plane features
    *  \param corespPlane, planeVect1, planeVect2, residuals, cost
    *  \return transformation vector
    */
    Eigen::VectorXd transformationPlane(Eigen::MatrixXi corespPlane,
                                       std::vector<Plane> planeVect1, std::vector<Plane> planeVect2,
                                       std::vector<double> *residuals, double *cost);

    /*!
    *  \brief apply transformation to the first scan
    * in order to determine the pose of the second scan
    *  \param transVec, scan1, scan2
    */
    void applyTransformation(Eigen::VectorXd transVec, std::shared_ptr<Scan> scan1, std::shared_ptr<Scan> scan2);

    /*!
    *  \brief a tool that builds a transformed scan to test the solver
    *  \param scan
    *  \return a transformed Scan
    */
    Scan testSolver(Scan scan);

};

#endif // TRANSFORMATIONSOLVER_H
