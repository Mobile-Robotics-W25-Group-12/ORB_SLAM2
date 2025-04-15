#include "MetricLogger.h"

#include <ctime>
#include <cstdlib>

#include "KeyFrame.h"
#include "Converter.h"
#include "KeyFrameDatabase.h"

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
  mLogDir = "logs/" + getTimestamp();
  system(("mkdir -p " + mLogDir).c_str());
  mLogFile = std::ofstream{mLogDir + "/log.csv"};
  mConnectedFramesFile = std::ofstream{mLogDir + "/connected_frames.csv"};
  mInitialCandidateFile = std::ofstream{mLogDir + "/initial_candidates.csv"};
  mFilteredCandidateFile = std::ofstream{mLogDir + "/filtered_candidates.csv"};
  mConsistentCandidateFile = std::ofstream{mLogDir + "/consistent_candidates.csv"};
  mParamsFile = std::ofstream{mLogDir + "/params.yaml"};
  // mTrajectoryFile = std::ofstream{log_dir + "trajectory.txt"}

  mLogFile << "id,numInitialCandidates,numFilteredCandidates,numAccFilteredCandidates,"
              "numConsistentCandidates,loopDetected,numMatched,"
              "matchedKf,computeSuccess,minScore,detectLoopMs,computeSimMs" << std::endl;
  // mConsistentCandidateFile << "id, frames" << std::endl;
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
      << mCurrentMetrics.matchedKf << ","
      << mCurrentMetrics.computeSuccess << ","
      << mCurrentMetrics.minScore << ","
      << mCurrentMetrics.detectLoopDurationMs << "," 
      << mCurrentMetrics.computeSimDurationMs << "\n";

    mActiveFrame = false;
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

void MetricLogger::trajectory(KeyFrame* pKF)
{
  cv::Mat R = pKF->GetRotation().t();
  vector<float> q = Converter::toQuaternion(R);
  cv::Mat t = pKF->GetCameraCenter();
  mTrajectoryFile << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
    << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << "\n";
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

void MetricLogger::minScore(float score)
{
  mCurrentMetrics.minScore = score;
}

void MetricLogger::connectedFrames(const std::set<ORB_SLAM2::KeyFrame*> connectedFrames) {
  if (connectedFrames.empty()) return;

  mConnectedFramesFile << mCurrentMetrics.frameId << ","; 
  for (auto& c : connectedFrames)
  {
    mConnectedFramesFile << c->mnFrameId << ",";
  }
  mConnectedFramesFile.seekp(-1, mConnectedFramesFile.cur);
  mConnectedFramesFile << "\n";
}

void MetricLogger::initialCandidates(const std::vector<ORB_SLAM2::KeyFrame*>& candidates)
{
  if (candidates.empty()) return;

  mInitialCandidateFile << mCurrentMetrics.frameId << ","; 
  for (auto& c : candidates)
  {
    mInitialCandidateFile << c->mnFrameId << "," << c->mLoopScore << ",";
  }
  mInitialCandidateFile.seekp(-1, mInitialCandidateFile.cur);
  mInitialCandidateFile << "\n";
}

void MetricLogger::filteredCandidates(const std::vector<ScoredKeyFrame>& candidates)
{
  if (candidates.empty()) return;

  mFilteredCandidateFile << mCurrentMetrics.frameId << ","; 
  for (auto& c : candidates)
  {
    mFilteredCandidateFile << c.kf->mnFrameId << "," << c.score << ",";
  }
  mFilteredCandidateFile.seekp(-1, mFilteredCandidateFile.cur);
  mFilteredCandidateFile << "\n";
}

void MetricLogger::consistentCandidates(const std::vector<ORB_SLAM2::KeyFrame*>& candidates)
{
  if (candidates.empty()) return;

  mConsistentCandidateFile << mCurrentMetrics.frameId << ","; 
  for (auto& c : candidates)
  {
    mConsistentCandidateFile << c->mnFrameId << "," << c->mLoopScore << ",";
  }
  mConsistentCandidateFile.seekp(-1, mConsistentCandidateFile.cur);
  mConsistentCandidateFile << "\n";
}

}
