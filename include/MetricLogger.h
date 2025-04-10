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
};

class MetricLogger
{
public:
  static MetricLogger& instance();

  void logFrame();
  void startFrame(int id);
  
  void numInitialCandidates(int n);
  void numFilteredCandidates(int n);
  void numAccFilteredCandidates(int n);
  void numConsistentCandidates(int n);
  void numMatched(int n);
  void loopDetected(bool detected);
  void computeSuccess(bool success);
  void matchedKf(int matchedKf);
  void minScore(float score);
  
  void consistentCandidates(const std::vector<ORB_SLAM2::KeyFrame*>& candidates);

private:
  MetricLogger();

  std::ofstream mLogFile;
  std::ofstream mConsistentCandidateFile;
  FrameMetrics mCurrentMetrics{};
  bool mActiveFrame{false};
};

}

#endif
