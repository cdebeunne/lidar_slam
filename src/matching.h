#ifndef MATCHING_H
#define MATCHING_H

#include "utility.h"
#include "scan.h"
#include "feature.h"

/*!
 * \file matching.h
 * \brief perform all the calculations to match features
 * \author César Debeunne
 */


class Matching
{
public:
    Matching();
    /*!
    *  \brief matches the planes of two scans
    *  \param planeVect1, planeVect2
    *  \return corespondencesPlane
    */
    Eigen::MatrixXi planeMatching(std::vector<Plane> planeVect1, std::vector<Plane> planeVect2);

    /*!
    *  \brief matches the edges of two scans
    *  \param edgeVect1, edgeVect2
    *  \return corespondencesEdge
    */
    Eigen::MatrixXi edgeMatching(std::vector<Edge> edgeVect1, std::vector<Edge> edgeVect2);

    static const float BARYTHRESHOLD; /*!< maximum distance between two features*/
    static const float EIGENTHRESHOLD_EDGE; /*!< maximum eigen distance between two edges*/
    static const float EIGENTHRESHOLD_PLANE; /*!< maximum eigen distance between two planes*/
    static const float ZTHRESHOLD_EDGE; /*!< the z parameter for outlier rejection*/
    static const float ZTHRESHOLD_PLANE;
};

#endif // MATCHING_H
