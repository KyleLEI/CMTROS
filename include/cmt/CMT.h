#ifndef CMT_H

#define CMT_H

#include "common.h"
#include "Consensus.h"
#include "Fusion.h"
#include "Matcher.h"
#include "Tracker.h"

#include <opencv2/core/cuda.hpp>
#include <opencv2/cudafeatures2d.hpp>

using cv::cuda::FastFeatureDetector;
using cv::cuda::ORB;
using cv::Ptr;
using cv::RotatedRect;
using cv::Size2f;

namespace cmt
{

class CMT
{
public:
    CMT(){};
    bool initialize(const GpuMat im_gray, const Rect rect);
    void processFrame(const GpuMat im_gray, const GpuMat im_prev);

    Fusion fusion;
    Matcher matcher;
    Tracker tracker;
    Consensus consensus;

    vector<Point2f> points_active; //public for visualization purposes
    RotatedRect bb_rot;

private:
    Ptr<FastFeatureDetector> detector;
    Ptr<ORB> descriptor;

    Size2f size_initial;

    vector<int> classes_active;

    float theta;
};

} /* namespace CMT */

#endif /* end of include guard: CMT_H */
