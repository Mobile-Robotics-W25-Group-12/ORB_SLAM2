#ifndef METRICLOGGER_H
#define METRICLOGGER_H

#include <fstream>
#include <string>
#include <ctime>

struct FrameMetrics
{
  int frameId{};
  int numInitialCandidates{};
  int numFilteredCandidates{};
  int numAccFilteredCandidates{};
  int numConsistentCandidates{};
  bool loopDetected{false};
  int numMatched{};
  bool computeSuccess{false};
  int matchedKf{};
};

class MetricLogger
{
public:
  void logFrame()
  {
    if (mActiveFrame)
    {
      mLogFile 
        << mCurrentMetrics.frameId << "," 
        << mCurrentMetrics.numInitialCandidates << "," 
        << mCurrentMetrics.numFilteredCandidates << "," 
        << mCurrentMetrics.numAccFilteredCandidates << "," 
        << mCurrentMetrics.numConsistentCandidates << "," 
        << mCurrentMetrics.loopDetected << "," 
        << mCurrentMetrics.numMatched << "," 
        << mCurrentMetrics.computeSuccess << ","
        << mCurrentMetrics.matchedKf << "\n";
      mActiveFrame = false;
      mLogFile.flush();
    }
  }

  void startFrame(int id)
  {
    if (mActiveFrame)
    {
      logFrame();
    }
    mCurrentMetrics = {};
    mCurrentMetrics.frameId = id;
    mActiveFrame = true;
  }
  
  void numInitialCandidates(int n)
  {
    mCurrentMetrics.numInitialCandidates = n;
  }
  
  void numFilteredCandidates(int n)
  {
    mCurrentMetrics.numFilteredCandidates = n;
  }
  
  void numAccFilteredCandidates(int n)
  {
    mCurrentMetrics.numAccFilteredCandidates = n;
  }
  
  void numConsistentCandidates(int n)
  {
    mCurrentMetrics.numConsistentCandidates = n;
  }
  
  void numMatched(int n)
  {
    mCurrentMetrics.numMatched = n;
  }

  void loopDetected(bool detected)
  {
    mCurrentMetrics.loopDetected = detected;
  }

  void computeSuccess(bool success)
  {
    mCurrentMetrics.computeSuccess = success;
  }

  void matchedKf(int matchedKf)
  {
    mCurrentMetrics.matchedKf = matchedKf;
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

  MetricLogger() : mLogFile("log_" + std::to_string(std::time(NULL)) + ".csv")
  {
    mLogFile << "id,numInitialCandidates,numFilteredCandidates,numAccFilteredCandidates,"
      "numConsistentCandidates,loopDetected,numMatched,"
      "computeSuccess,matchedKf\n";
  }
};

#endif