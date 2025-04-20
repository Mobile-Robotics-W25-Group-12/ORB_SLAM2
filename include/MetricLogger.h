#ifndef METRICLOGGER_H
#define METRICLOGGER_H

#include <fstream>
#include <string>
#include <vector>
#include <set>

#include "LoopClosureConfig.h"

namespace ORB_SLAM2
{

class KeyFrame;
class ScoredKeyFrame;

struct FrameMetrics
{
  int frameId{};
  int numInitialCandidates{};
  int numFilteredCandidates{};
  int numAccFilteredCandidates{};
  int numConsistentCandidates{};
  bool loopDetected{};
  int numMatchedFrames{};
  bool ransacPoseEstimateSolved{};
  bool poseOptimized{};
  int matchedKf{};
  bool computeSuccess{};
  float minScore{};
  float detectLoopDurationMs{};
  float computeSimDurationMs{};
};

class MetricLogger
{
public:
  static MetricLogger& instance();

  void logFrame();
  void startFrame(int id);
  
  void trajectory(KeyFrame* kf);

  void flush()
  {
    mLogFile.flush();
    mFilteredCandidateFile.flush();
    mConsistentCandidateFile.flush();
    mParamsFile.flush();
    mTrajectoryFile.flush();
  }
  
  void numInitialCandidates(int n) { mCurrentMetrics.numInitialCandidates = n; }
  void numFilteredCandidates(int n) { mCurrentMetrics.numFilteredCandidates = n; }
  void numAccFilteredCandidates(int n) { mCurrentMetrics.numAccFilteredCandidates = n; }
  void numConsistentCandidates(int n) { mCurrentMetrics.numConsistentCandidates = n; }
  void loopDetected(bool detected) { mCurrentMetrics.loopDetected = detected; }
  void numMatchedFrames(int n) { mCurrentMetrics.numMatchedFrames = n; }
  void ransacPoseEstimateSolved(bool success) { mCurrentMetrics.ransacPoseEstimateSolved = success; }
  void poseOptimized(bool success) { mCurrentMetrics.poseOptimized = success; }
  void matchedKf(int n) { mCurrentMetrics.matchedKf = n; }
  void computeSuccess(bool success) { mCurrentMetrics.computeSuccess = success; }
  void minScore(float score) { mCurrentMetrics.minScore = score; }
  void detectLoopDurationMs(float ms) { mCurrentMetrics.detectLoopDurationMs = ms; }
  void computeSimDurationMs(float ms) { mCurrentMetrics.computeSimDurationMs = ms; }

  void connectedFrames(const std::set<KeyFrame*> connectedFrames);
  void initialCandidates(const std::vector<ORB_SLAM2::KeyFrame*>& candidates);
  void filteredCandidates(const std::vector<ScoredKeyFrame>& candidates);
  void accFilteredCandidates(const std::vector<ORB_SLAM2::KeyFrame*>& candidates);
  void consistentCandidates(const std::vector<ORB_SLAM2::KeyFrame*>& candidates);
  void matchedFrames(const std::vector<ORB_SLAM2::KeyFrame*>& candidates);
  void ransacSolvedFrames(const std::set<ORB_SLAM2::KeyFrame*>& candidates);

  void logParams(LoopClosureConfig &config) {
    mParamsFile << "useVectorScores: " << config.useVectorScores << "\n";
    mParamsFile << "minVectorScore: " << config.minVectorScore << "\n";
    mParamsFile << "numInitialMatchPoints: " << config.numInitialMatchPoints << "\n";
    mParamsFile << "numRansacInliers: " << config.numRansacInliers << "\n";
    mParamsFile << "numOptimizationInliers: " << config.numOptimizationInliers << "\n";
    mParamsFile << "numProjectedMatchPoints: " << config.numProjectedMatchPoints<< "\n";
    mParamsFile << "projTreshold: " << config.projTreshold << "\n";
    mParamsFile << "covisibilityConsistencyTh: " << config.covisibilityConsistencyTh << "\n";
  }

  void logIsStereo(bool isStereo)
  { 
    mParamsFile << "isStereo: " << isStereo << std::endl;
  }

  void logVectorFilepath(const std::string& filepath)
  { 
    mParamsFile << "vectorFilepath: " << filepath << std::endl;
  }

  void logDatasetPath(const std::string& path)
  { 
    mParamsFile << "datasetPath: " << path << std::endl;
  }

  std::string getLogDirectory() {
    return mLogDir;
  }

private:
  MetricLogger();

  std::string mLogDir;
  std::ofstream mLogFile;
  std::ofstream mConnectedFramesFile;
  std::ofstream mInitialCandidateFile;
  std::ofstream mFilteredCandidateFile;
  std::ofstream mAccFilteredCandidateFile;
  std::ofstream mConsistentCandidateFile;
  std::ofstream mMatchedFramesFile;
  std::ofstream mRansacSolvedFramesFile;
  std::ofstream mParamsFile;
  std::ofstream mTrajectoryFile;
  FrameMetrics mCurrentMetrics{};
  bool mActiveFrame{false};
};

}

#endif
