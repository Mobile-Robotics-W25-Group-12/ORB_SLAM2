/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <vector>
#include <fstream>
#include <list>
#include <set>

#include "MetricLogger.h"
#include "KeyFrame.h"
#include "Frame.h"
#include "ORBVocabulary.h"

#include "VectorDatabase.h"

#include "LoopClosureConfig.h"

#include <mutex>

#define LOOP_CANDIDATES_DEFAULT // change to our custom method(s)

namespace ORB_SLAM2
{

class KeyFrame;
class Frame;
class MetricLogger;

struct ScoredKeyFrame {
  float score;
  KeyFrame *kf;
};

class KeyFrameDatabase
{
public:

    KeyFrameDatabase(const ORBVocabulary &voc);

   void add(KeyFrame* pKF);

   void erase(KeyFrame* pKF);

   void clear();

   // Loop Detection
   std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame* pKF, float minScore);

   // Custom loop detection
   float Score(KeyFrame* kf1, KeyFrame* kf2);

   float MinScore(KeyFrame *kf);

   std::vector<KeyFrame *> CustomDetectLoopCandidates(KeyFrame* kf, float minScore);
   std::vector<KeyFrame*> QueryCandidates(KeyFrame* kf, std::set<KeyFrame*> connectedFrames, float minScore);
   std::vector<ScoredKeyFrame> FilterLoopCandidates(KeyFrame* kf, const std::vector<KeyFrame *>& candidates, float minScore, float minCommonWords);
   std::vector<KeyFrame *> AccumlatedFilterLoopCandidates(KeyFrame* kf, const std::vector<ScoredKeyFrame>& candidates, float minScore, float minCommonWords);
   
   // Relocalization
   std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F);

protected:

  // Associated vocabulary
  const ORBVocabulary* mpVoc;

  // Inverted file
  std::vector<list<KeyFrame*> > mvInvertedFile;

  VectorDatabase<float> mVectorDb;

  LoopClosureConfig &loopClosureConfig; 

  // Mutex
  std::mutex mMutex;
};

} //namespace ORB_SLAM

#endif
