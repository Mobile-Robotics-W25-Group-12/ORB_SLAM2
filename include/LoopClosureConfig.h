#pragma once

#include <fstream>
#include <fkYAML/node.hpp>

namespace ORB_SLAM2
{    

class LoopClosureConfig
{
public:
    LoopClosureConfig() {
        std::ifstream in{"loop_closure_params.yaml"};
        auto node = fkyaml::node::deserialize(in);

        vectorFilepath = node.at("vectorFilepath").as_str();
        minVectorScore = node.at("minVectorScore").as_float();
        useVectorScores = node.at("useVectorScores").as_bool();
        numInitialMatchPoints = node.at("numInitialMatchPoints").as_int();
        numRansacInliers = node.at("numRansacInliers").as_int();
        numOptimizationInliers = node.at("numOptimizationInliers").as_int();
        projTreshold = node.at("projTreshold").as_int();
        numProjectedMatchPoints = node.at("numProjectedMatchPoints").as_int();
        covisibilityConsistencyTh = node.at("covisibilityConsistencyTh").as_int();

        if(!useVectorScores && minVectorScore > 0.1) {
            throw std::runtime_error("When useVectorScores: 0, minVectorScore should not be very large");
        }
    }

    static LoopClosureConfig &instance() {
        static LoopClosureConfig inst{};
        return inst;
    }

    std::string vectorFilepath;
    bool useVectorScores = false;
    float minVectorScore;
    int numInitialMatchPoints;
    int numRansacInliers;
    int numOptimizationInliers;
    int projTreshold;
    int numProjectedMatchPoints;
    int covisibilityConsistencyTh;
};

}