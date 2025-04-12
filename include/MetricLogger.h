#ifndef METRICLOGGER_H
#define METRICLOGGER_H

#include <fstream>
#include <string>
#include <vector>

namespace ORB_SLAM2
{

class KeyFrame;

struct FrameMetrics
{
  int frameId{};
  int numInitialCandidates{};
  int numFilteredCandidates{};
  int numAccFilteredCandidates{};
  int numConsistentCandidates{};
  bool loopDetected{false};
  int numMatched{};
  int matchedKf{};
  bool poseEstimated{false};
  bool computeSuccess{false};
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
  void numInitialCandidates(int n);
  void numFilteredCandidates(int n);
  void numAccFilteredCandidates(int n);
  void numConsistentCandidates(int n);
  void numMatched(int n);
  void loopDetected(bool detected);
  void computeSuccess(bool success);
  void matchedKf(int matchedKf);
  void minScore(float score);

  void flush()
  {
    mLogFile.flush();
    mConsistentCandidateFile.flush();
    mParamsFile.flush();
    mTrajectoryFile.flush();
  }
  
  void detectLoopDuration(float ms)
  {
    mCurrentMetrics.detectLoopDurationMs = ms;
  }

  void computeSimDuration(float ms)
  {
    mCurrentMetrics.computeSimDurationMs = ms;
  }
  
  void consistentCandidates(const std::vector<ORB_SLAM2::KeyFrame*>& candidates);

  void logParams(bool useVectorScores, int numRansacInliers, int projTreshold, int numMatchPoints) {
    mParamsFile << "useVectorScores = " << useVectorScores << "\n";
    mParamsFile << "numRansacInliers = " << numRansacInliers << "\n";
    mParamsFile << "projTreshold = " << projTreshold << "\n";
    mParamsFile << "numMatchPoints = " << numMatchPoints << std::endl;    
  }

  void logVectorFilepath(const std::string& filepath)
  { 
    mParamsFile << "vectorFilepath = " << filepath << std::endl;
  }

  void logDatasetPath(const std::string& path)
  { 
    mParamsFile << "datasetPath = " << path << std::endl;
  }

  std::string getLogDirectory() {
    return mLogDir;
  }

private:
  MetricLogger();

  std::string mLogDir;
  std::ofstream mLogFile;
  std::ofstream mConsistentCandidateFile;
  std::ofstream mParamsFile;
  std::ofstream mTrajectoryFile;
  FrameMetrics mCurrentMetrics{};
  bool mActiveFrame{false};
};

}

#endif
