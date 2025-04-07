#include "MetricLogger.h"

#include <ctime>
#include <cstdlib>

#include "KeyFrame.h"

static std::string getTimestamp() {
    std::time_t now = std::time(nullptr);
    std::tm* tm_now = std::localtime(&now);
    std::ostringstream oss;
    oss << std::put_time(tm_now, "%Y%m%d_%H%M%S");
    return oss.str();
}

namespace ORB_SLAM2
{

MetricLogger& MetricLogger::instance()
{
  static MetricLogger inst{};
  return inst;
}

MetricLogger::MetricLogger()
{
  std::string log_dir = "logs/" + getTimestamp();
  system(("mkdir -p " + log_dir).c_str());
  mLogFile = std::ofstream{log_dir + "/log.csv"};
  mConsistentCandidateFile = std::ofstream{log_dir + "/consistent_candidates.csv"};

  mLogFile << "id,numInitialCandidates,numFilteredCandidates,numAccFilteredCandidates,"
              "numConsistentCandidates,loopDetected,numMatched,"
              "computeSuccess,matchedKf" << std::endl;
  mConsistentCandidateFile << "id", "frames" << std::endl;
}

void MetricLogger::logFrame()
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

void MetricLogger::startFrame(int id)
{
  if (mActiveFrame)
  {
    logFrame();
  }
  mCurrentMetrics = {};
  mCurrentMetrics.frameId = id;
  mActiveFrame = true;
}

void MetricLogger::numInitialCandidates(int n)
{
  mCurrentMetrics.numInitialCandidates = n;
}

void MetricLogger::numFilteredCandidates(int n)
{
  mCurrentMetrics.numFilteredCandidates = n;
}

void MetricLogger::numAccFilteredCandidates(int n)
{
  mCurrentMetrics.numAccFilteredCandidates = n;
}

void MetricLogger::numConsistentCandidates(int n)
{
  mCurrentMetrics.numConsistentCandidates = n;
}

void MetricLogger::numMatched(int n)
{
  mCurrentMetrics.numMatched = n;
}

void MetricLogger::loopDetected(bool detected)
{
  mCurrentMetrics.loopDetected = detected;
}

void MetricLogger::computeSuccess(bool success)
{
  mCurrentMetrics.computeSuccess = success;
}

void MetricLogger::matchedKf(int matchedKf)
{
  mCurrentMetrics.matchedKf = matchedKf;
}

void MetricLogger::consistentCandidates(const std::vector<ORB_SLAM2::KeyFrame*>& candidates)
{
  if (candidates.empty()) return;

  mConsistentCandidateFile << mCurrentMetrics.frameId << ","; 
  for (auto& c : candidates)
  {
    mConsistentCandidateFile << c->mnFrameId << ",";
  }
  mConsistentCandidateFile << std::endl;
}

}
