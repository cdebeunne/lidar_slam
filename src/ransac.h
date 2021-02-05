#ifndef RANSAC_H
#define RANSAC_H
#include <Eigen/Dense>
#include "scan.h"
#include "transformationsolver.h"

/*!
 * \file ransac.h
 * \brief a basic implementation of ransac algorithm adapted
 * to our problem
 * \author César Debeunne
 */


class RANSAC
{
public:
    RANSAC(int setSize, int minResiduals, float residualThreshold, double costThreshold);

    /*!
    *  \brief ransac algorithm for edge features
    *  \param two scans, an initial corespondence matrix, a pointer to a
    * translation vector
    *  \return a corespondence matrix
    */
    Eigen::MatrixXi ransacFilterEdge(std::shared_ptr<Scan> scan1, std::shared_ptr<Scan> scan2,
                                     Eigen::MatrixXi corespEdge, Eigen::VectorXd *transVec);

    /*!
    *  \brief ransac algorithm for plane features
    *  \param two scans, an initial corespondence matrix, a pointer to a
    * translation vector
    *  \return a corespondence matrix
    */
    Eigen::MatrixXi ransacFilterPlane(std::shared_ptr<Scan> scan1, std::shared_ptr<Scan> scan2,
                                      Eigen::MatrixXi corespPlane, Eigen::VectorXd *transVec);

private:
    /*!
    *  \brief a function that returns a random set of corespondences
    *  \param the corespondence matrix and an integer to feed the random int generator
    *  \return a corespondence matrix
    */
    Eigen::MatrixXi randomCoresp(Eigen::MatrixXi corespondences, int k);

private:
    TransformationSolver _solver;
    int _setSize;
    int _minResiduals;
    float _residualThreshold;
    double _pInlier;
    double _costTheshold;

};

#endif // RANSAC_H
