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
        useVectorScores = node.at("useVectorScores").as_bool();
        numRansacInliers = node.at("numRansacInliers").as_int();
        projTreshold = node.at("projTreshold").as_int();
        numMatchPoints = node.at("numMatchPoints").as_int();
    }

    static LoopClosureConfig &instance() {
        static LoopClosureConfig inst{};
        return inst;
    }

    std::string vectorFilepath;
    bool useVectorScores = false;
    int numRansacInliers;
    int projTreshold;
    int numMatchPoints;
};

}