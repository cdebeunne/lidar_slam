#include "ransac.h"
#include <math.h>

RANSAC::RANSAC(int setSize, int minResiduals, float residualThreshold, double costThreshold):
    _setSize(setSize), _minResiduals(minResiduals), _residualThreshold(residualThreshold),
    _costTheshold(costThreshold)
{
    _pInlier = 0.5;
}

Eigen::MatrixXi RANSAC::randomCoresp(Eigen::MatrixXi corespondences, int k){
    std::vector<int> randomIndex;
    Eigen::MatrixXi newCoresp(_setSize, 2);
    int idx;

    // generate the random engine
    srand(static_cast<int>(time(0)) + k);

    idx = rand() % (corespondences.rows()-1);
    randomIndex.push_back(idx);
    newCoresp(0,0) = corespondences(idx,0);
    newCoresp(0,1) = corespondences(idx,1);

    for (int i=1; i<_setSize; i++){
        idx = rand() % (corespondences.rows()-1);
        while (std::find(randomIndex.begin(), randomIndex.end(), idx) != randomIndex.end()){
            idx = rand() % (corespondences.rows()-1);
        }
        randomIndex.push_back(idx);
        newCoresp(i,0) = corespondences(idx,0);
        newCoresp(i,1) = corespondences(idx,1);
    }
    return newCoresp;
}

Eigen::MatrixXi RANSAC::ransacFilterEdge(std::shared_ptr<Scan> scan1, std::shared_ptr<Scan> scan2,
                                         Eigen::MatrixXi corespEdge,Eigen::VectorXd *transVec)
{
    double cost;
    std::vector<double> residuals;
    double bestCost = 1;
    Eigen::VectorXd tempTransVec;
    int i = 0;
    double p = 0.05;
    Eigen::MatrixXi finalCoresp = corespEdge;
    double numIter = std::min(std::log(1-p)/std::log(1-std::pow(_pInlier,_setSize)), (double)100);

    // main loop of RANSAC
    while (bestCost > _costTheshold && i < numIter){
        i++;
        Eigen::MatrixXi newCoresp = randomCoresp(corespEdge, i);
        tempTransVec = _solver.transformationEdge(newCoresp,
                                                  scan1->getEdgeVect(), scan2->getEdgeVect(),
                                                  &residuals, &cost);
        if (cost < bestCost){
            *transVec = tempTransVec;
            bestCost = cost;
            finalCoresp = newCoresp;
        }

        // here we check the residuals
        std::vector<int> newSet;
        for (int k=0; k<_setSize; k++){
            if (residuals[k] < _residualThreshold){
                newSet.push_back(k);
            }
        }

        // if the size is big enough, we may have found the best set
        if (static_cast<int>(newSet.size()) > _minResiduals){
            Eigen::MatrixXi filteredCoresp(newSet.size(), 2);
            for (int k=0; k<newSet.size(); k++){
                filteredCoresp(k,0) = newCoresp(newSet[k],0);
                filteredCoresp(k,1) = newCoresp(newSet[k],1);
            }
            tempTransVec = _solver.transformationEdge(filteredCoresp,
                                                      scan1->getEdgeVect(), scan2->getEdgeVect(),
                                                      &residuals, &cost);
            if (cost/newSet.size() < bestCost){
                *transVec = tempTransVec;
                bestCost = cost/newSet.size();
                finalCoresp = filteredCoresp;
                _pInlier = (double)newSet.size()/_setSize;
            }
        }
    }
    return finalCoresp;
}

Eigen::MatrixXi RANSAC::ransacFilterPlane(std::shared_ptr<Scan> scan1, std::shared_ptr<Scan> scan2,
                               Eigen::MatrixXi corespPlane, Eigen::VectorXd *transVec)
{
    double cost;
    std::vector<double> residuals;
    double bestCost = 1;
    Eigen::VectorXd tempTransVec;
    Eigen::MatrixXi finalCoresp = corespPlane;
    int i = 0;
    double numIter = 100;

    // main loop of RANSAC
    while (bestCost > _costTheshold && i < numIter){
        i++;
        Eigen::MatrixXi newCoresp = randomCoresp(corespPlane, i);
        tempTransVec = _solver.transformationPlane(newCoresp,
                                                  scan1->getPlaneVect(), scan2->getPlaneVect(),
                                                  &residuals, &cost);

        // here we check the residuals
        std::vector<int> newSet;
        for (int k=0; k<_setSize; k++){
            if (residuals[k] < _residualThreshold){
                newSet.push_back(k);
            }
        }

        // if the size is big enough, we may have found the best set
        if (newSet.size() > _minResiduals){
            Eigen::MatrixXi filteredCoresp(newSet.size(), 2);
            for (int k=0; k<newSet.size(); k++){
                filteredCoresp(k,0) = newCoresp(newSet[k],0);
                filteredCoresp(k,1) = newCoresp(newSet[k],1);
            }
            tempTransVec = _solver.transformationPlane(filteredCoresp,
                                                      scan1->getPlaneVect(), scan2->getPlaneVect(),
                                                      &residuals, &cost);
            if (cost/newSet.size() < bestCost){
                *transVec = tempTransVec;
                bestCost = cost/newSet.size();
                _pInlier = (double)newSet.size()/_setSize;
                finalCoresp = filteredCoresp;
            }
        }
    }
    return finalCoresp;
}
