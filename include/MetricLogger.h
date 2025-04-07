#ifndef METRICLOGGER_H
#define METRICLOGGER_H

#include <fstream>
#include <string>
#include <ctime>

struct FrameMetrics
{
  int frameId{};
  int numFinalCandidates{};
  bool loopDetected{false};
  bool computeSuccess{false};
};

class MetricLogger
{
public:
  void logFrame()
  {
    if (mActiveFrame)
    {
      mLogFile << mCurrentMetrics.frameId << "," << mCurrentMetrics.numFinalCandidates << "," << mCurrentMetrics.loopDetected << mCurrentMetrics.computeSuccess << "\n";
      mActiveFrame = false;
      mLogFile.flush();
    }
  }

  void startFrame(int id)
  {
    if (mActiveFrame)
    {
      logFrame();
      mCurrentMetrics = {};
    }
    mCurrentMetrics.frameId = id;
    mActiveFrame = true;
  }
  
  void numFinalCandidates(int n)
  {
    mCurrentMetrics.numFinalCandidates = n;
  }

  void loopDetected(bool detected)
  {
    mCurrentMetrics.loopDetected = detected;
  }

  void computeSuccess(bool success)
  {
    mCurrentMetrics.computeSuccess = success;
  }

  static MetricLogger& instance()
  {
    static MetricLogger inst{};
    return inst;
  }

private:
  std::ofstream mLogFile;
  FrameMetrics mCurrentMetrics{};
  bool mActiveFrame{false};

  MetricLogger() : mLogFile(std::to_string(std::time(NULL)) + "_log.csv")
  {
    mLogFile << "id,numFinalCandidates,loopDetected,computeSuccess\n";
  }
};

#endif