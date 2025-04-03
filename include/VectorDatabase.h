#ifndef VECTORDATABASEH
#define VECTORDATABASEH

#include "npy.hpp"
#include <vector>
#include <string>
#include <functional>


template<typename T>
class VectorDatabase {

using vector_iterator = typename std::vector<T>::iterator;
using score_func_t = std::function<float(const std::vector<T> &, const std::vector<T> &)>;

public:
    void LoadNumpyVectors(const std::string &path) {
        npy::npy_data<T> data = npy::read_npy<T>(path);
        int numFrames = data.shape[0];
        int dimensions = data.shape[1];
        mAllVectors.reserve(numFrames);
        for(int i = 0; i < numFrames; ++i) {
            mAllVectors.emplace_back(data.data.begin() + i*dimensions, data.data.begin() + (i+1)*dimensions);
        }
    }

    const std::vector<T>& GetNumpyVector(int frameId) {
        return mAllVectors[frameId];
    }

    void AddKeyframeVector(int frameId, const std::vector<T> &v) {
        mVectors[frameId] = v;

        GetAllScores(v, VectorDatabase::CosineSimilarity);
    }

    std::unordered_map<int, float> GetAllScores(const std::vector<T> &v, score_func_t scoreFunc) {
        std::unordered_map<int, float> scores;
        for(auto &p : mVectors) {
            scores[p.first] = scoreFunc(v, p.second);
        }
        return scores;
    }

    static float CosineSimilarity(const std::vector<T> &a, const std::vector<T> &b) {
        assert(a.size() == b.size());
        double dot = 0.0, denom_a = 0.0, denom_b = 0.0 ;
        for(int i = 0; i < a.size(); ++i) {
            dot += a[i] * b[i];
            denom_a += a[i] * a[i];
            denom_b += b[i] * b[i];
        }
        return dot / (sqrt(denom_a) * sqrt(denom_b)) ;
    }

private:
    std::vector<std::vector<T>> mAllVectors;

    std::unordered_map<int, std::vector<T>> mVectors;
};

#endif
