#include "matching.h"
#include "scan.h"
#include "feature.h"

#include <Eigen/Dense>
#include <math.h>

const float Matching::BARYTHRESHOLD = 1.5;
const float Matching::EIGENTHRESHOLD_EDGE = 30;
const float Matching::EIGENTHRESHOLD_PLANE = 100;
const float Matching::ZTHRESHOLD_EDGE = 0.5;
const float Matching::ZTHRESHOLD_PLANE = 0.5;

std::vector<int> outlierRejection(std::vector<float> distList, float zThreshold){
    float mu=0;
    float sigma=0;
    float z;

    std::vector<int> inliers;

    for (auto it:distList){
        mu = mu+it/distList.size();
    }
    for (auto it:distList){
        sigma = sigma+pow(it-mu, 2);
    }
    sigma = pow(sigma, 0.5);

    //perfect case
    if (sigma<0.01){
        for (auto it:distList){
            inliers.push_back(1);
        }
        return inliers;
    }

    for (auto it:distList){
        z = fabs((it-mu)/sigma);
        if (z<zThreshold){
            inliers.push_back(1);
        }else{
            inliers.push_back(0);
        }
    }
    return inliers;
}

Matching::Matching()
{
}

Eigen::MatrixXi Matching::edgeMatching(std::vector<Edge> edgeVect1, std::vector<Edge> edgeVect2)
{
    Eigen::Vector3f ev1;
    Eigen::Vector3f ev2;
    Eigen::Vector3f barycenter1;
    Eigen::Vector3f barycenter2;
    Eigen::Vector3f dir1;
    Eigen::Vector3f dir2;
    int size1;
    int size2;
    std::vector<int> corespTemp;
    float sizeDelta;
    float eigDist;
    float dirDist;
    float centerDist;
    float baryDist;
    int idx[2];

    // building the distance matrix
    Eigen::ArrayXXf distMat(edgeVect1.size(), edgeVect2.size());
    for (int i=0; i<static_cast<int>(edgeVect1.size()); i++){
        for (int j=0; j<static_cast<int>(edgeVect2.size()); j++){
            ev1 = edgeVect1[i].getEigenValues();
            dir1 = edgeVect1[i].getDirection();
            barycenter1 = edgeVect1[i].getBarycenter();
            size1 = edgeVect1[i].getSize();
            ev2 = edgeVect2[j].getEigenValues();
            dir2 = edgeVect2[j].getDirection();
            barycenter2 = edgeVect2[j].getBarycenter();
            size2 = edgeVect2[j].getSize();

            sizeDelta = std::abs(size1-size2)/std::max(size1,size2);
            eigDist = (ev2-ev1).norm();
            dirDist = std::abs(dir1.dot(dir2)-dir1.norm()*dir2.norm());
            centerDist = std::abs(barycenter1(2)-barycenter2(2));
            baryDist = (barycenter1.head(2)-barycenter2.head(2)).norm();

            if (baryDist > BARYTHRESHOLD){
                distMat(i,j) = 100;
            } else{
                distMat(i,j) = eigDist + dirDist;
            }
        }
    }

    // establishing the corespondences
    Eigen::Index index2;
    float minVal1;
    float minVal2;
    for(int i=0;i<distMat.rows();++i){
        minVal1 = distMat.row(i).minCoeff(&index2);
        minVal2 = distMat.col(index2).minCoeff();
        if (minVal1 == minVal2 && minVal1 < EIGENTHRESHOLD_EDGE){
            idx[0] = i;
            idx[1] = index2;
            corespTemp.push_back(idx[0]);
            corespTemp.push_back(idx[1]);
        }
    }

    // building the corespondance matrix
    Eigen::MatrixXi corespondanceEdge(corespTemp.size()/2, 2);
    for (int i=0; i<static_cast<int>(corespTemp.size()/2); i++){
        corespondanceEdge(i,0) = corespTemp.at(i*2);
        corespondanceEdge(i,1) = corespTemp.at(i*2+1);
    }
    return corespondanceEdge;

}

Eigen::MatrixXi Matching::planeMatching(std::vector<Plane> planeVect1, std::vector<Plane> planeVect2)
{
    Eigen::Vector3f ev1;
    Eigen::Vector3f ev2;
    Eigen::Vector3f barycenter1;
    Eigen::Vector3f barycenter2;
    Eigen::Vector3f dir1;
    Eigen::Vector3f dir2;
    int size1;
    int size2;
    std::vector<int> corespTemp;
    float sizeDelta;
    float eigDist;
    float dirDist;
    float centerDist;
    float baryDist;
    int idx[2];

    // building the distance matrix
    Eigen::ArrayXXf distMat(planeVect1.size(), planeVect2.size());
    for (int i=0; i<static_cast<int>(planeVect1.size()); i++){
        for (int j=0; j<static_cast<int>(planeVect2.size()); j++){
            ev1 = planeVect1[i].getEigenValues();
            dir1 = planeVect1[i].getDirection();
            barycenter1 = planeVect1[i].getBarycenter();
            size1 = planeVect1[i].getSize();
            ev2 = planeVect2[j].getEigenValues();
            dir2 = planeVect2[j].getDirection();
            barycenter2 = planeVect2[j].getBarycenter();
            size2 = planeVect2[j].getSize();

            sizeDelta = std::abs(size1-size2)/std::max(size1,size2);
            eigDist = (ev2-ev1).norm();
            dirDist = std::abs(dir1.dot(dir2)-dir1.norm()*dir2.norm());
            centerDist = std::abs(barycenter1(2)-barycenter2(2));
            baryDist = (barycenter1.head(2)-barycenter2.head(2)).norm();

            if (baryDist > BARYTHRESHOLD){
                distMat(i,j) = 100;
            } else{
                distMat(i,j) = dirDist+sizeDelta;
            }
        }
    }

    // establishing the corespondences
    Eigen::Index index2;
    float minVal1;
    float minVal2;
    for(int i=0;i<distMat.rows();++i){
        minVal1 = distMat.row(i).minCoeff(&index2);
        minVal2 = distMat.col(index2).minCoeff();
        if (minVal1 == minVal2 && minVal1 < EIGENTHRESHOLD_PLANE){
            idx[0] = i;
            idx[1] = index2;
            corespTemp.push_back(idx[0]);
            corespTemp.push_back(idx[1]);
        }
    }

    // building the corespondance matrix
    Eigen::MatrixXi corespondancePlane(corespTemp.size()/2, 2);
    for (int i=0; i<static_cast<int>(corespTemp.size()/2); i++){
        corespondancePlane(i,0) = corespTemp.at(i*2);
        corespondancePlane(i,1) = corespTemp.at(i*2+1);
    }
    return corespondancePlane;
}

