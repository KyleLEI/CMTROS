#ifndef TRACKER_H

#define TRACKER_H

#include "common.h"
#include <opencv2/cudaoptflow.hpp>

namespace cmt {

class Tracker
{
public:
    Tracker() : thr_fb(30) , cuda_of(cv::cuda::SparsePyrLKOpticalFlow::create()){};
    void track(const cv::cuda::GpuMat im_prev, const cv::cuda::GpuMat im_gray, 
            const vector<Point2f> & points_prev, vector<Point2f> & points_tracked,
            vector<unsigned char> & status);


private:
    float thr_fb;
    cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> cuda_of;
};

} /* namespace CMT */

#endif /* end of include guard: TRACKER_H */
