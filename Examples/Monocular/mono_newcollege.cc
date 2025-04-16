/**
 * Monocular loader for the “newCollege” dataset
 * Dataset layout expected:
 *   <sequence>/
 *      images/
 *         infra1_<sec>_<nsec>.png   (IR–grayscale left camera)
 *
 * The timestamp is parsed from the file name, so no times.txt is needed.
 */

 #include<iostream>
 #include<algorithm>
 #include<fstream>
 #include<chrono>
 #include<iomanip>
 #include<dirent.h>
 #include<cstring>
 #include <unistd.h>
 #include <stdio.h>
 #include <stdlib.h>
 
 #include<opencv2/core/core.hpp>
 
 #include "System.h"
 #include "MetricLogger.h"
 
 using namespace std;
 
 void LoadImages(const string &strSequence,
                 vector<string> &vstrImageFilenames,
                 vector<double> &vTimestamps);
 
 // ────────────────────────────────────────────────────────────────
 int main(int argc, char **argv)
 {
     if(argc != 4)
     {
         cerr << endl
              << "Usage: ./mono_newcollege path_to_vocabulary "
              << "path_to_settings path_to_sequence" << endl;
         return 1;
     }
 
     /* 1. Retrieve image paths and timestamps */
     vector<string> vstrImageFilenames;
     vector<double> vTimestamps;
     LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);
 
     int nImages = vstrImageFilenames.size();
     if(nImages==0){
         cerr << "No images found in "<< argv[3] << "/images" << endl;
         return 1;
     }
 
     ORB_SLAM2::MetricLogger::instance().logDatasetPath(string(argv[3]));
 
     /* 2. Create SLAM system */
     ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
 
     vector<float> vTimesTrack(nImages);
 
     cout << endl << "-------" << endl;
     cout << "Start processing sequence ..." << endl;
     cout << "Images in the sequence: " << nImages << endl << endl;
 
     cv::Mat im;
     for(int ni=0; ni<nImages; ni++)
     {
         im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
         double tframe = vTimestamps[ni];
 
         if(im.empty())
         {
             cerr << endl << "Failed to load image at: "
                  << vstrImageFilenames[ni] << endl;
             return 1;
         }
 
 #ifdef COMPILEDWITHC11
         std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
 #else
         std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
 #endif
 
         SLAM.TrackMonocular(im,tframe);
 
 #ifdef COMPILEDWITHC11
         std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
 #else
         std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
 #endif
 
         double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
         vTimesTrack[ni]=ttrack;
 
         /* Real‑time playback pacing */
         double T=0;
         if(ni<nImages-1)
             T = vTimestamps[ni+1]-tframe;
         else if(ni>0)
             T = tframe-vTimestamps[ni-1];
 
         if(ttrack<T)
             usleep((T-ttrack)*1e6);
     }
 
     SLAM.Shutdown();
 
     /* Timing statistics */
     sort(vTimesTrack.begin(),vTimesTrack.end());
     float totaltime = accumulate(vTimesTrack.begin(), vTimesTrack.end(), 0.0f);
 
     cout << "-------" << endl << endl;
     cout << "median tracking time: " << vTimesTrack[nImages/2] << " s" << endl;
     cout << "mean   tracking time: " << totaltime/nImages   << " s" << endl;
 
     SLAM.SaveKeyFrameTrajectoryTUM(
         ORB_SLAM2::MetricLogger::instance().getLogDirectory() +
         "/KeyFrameTrajectory_newCollege.txt");
 
     return 0;
 }
 
 // ────────────────────────────────────────────────────────────────
 void LoadImages(const string &strPathToSequence,
                 vector<string> &vstrImageFilenames,
                 vector<double> &vTimestamps)
 {
     const string imgDir = strPathToSequence + "/images";
     DIR *dir = opendir(imgDir.c_str());
     if(!dir){
         cerr << "Cannot open directory: " << imgDir << endl;
         return;
     }
 
     struct dirent *ent;
     struct FileEntry { string name; double ts; };
     vector<FileEntry> files;
 
     while((ent = readdir(dir)) != nullptr)
     {
         string fname(ent->d_name);
         if(fname.size()<11) continue;                       // skip . .. etc.
         if(fname.compare(0,7,"infra1_")!=0) continue;       // keep only left cam
 
         /* Parse timestamp: infra1_<sec>_<nsec>.png */
         size_t firstUnd = fname.find('_',7);
         size_t dot      = fname.find('.', firstUnd);
         if(firstUnd==string::npos || dot==string::npos) continue;
 
         double sec  = atof(fname.substr(7, firstUnd-7).c_str());
         double nsec = atof(fname.substr(firstUnd+1, dot-firstUnd-1).c_str());
         double ts   = sec + nsec*1e-9;
 
         files.push_back({fname, ts});
     }
     closedir(dir);
 
     /* sort by timestamp */
     sort(files.begin(), files.end(),
          [](const FileEntry& a, const FileEntry& b){ return a.ts < b.ts; });
 
     vstrImageFilenames.reserve(files.size());
     vTimestamps.reserve(files.size());
 
     for(const auto& f : files){
         vstrImageFilenames.push_back(imgDir + "/" + f.name);
         vTimestamps.push_back(f.ts);
     }
 }
 