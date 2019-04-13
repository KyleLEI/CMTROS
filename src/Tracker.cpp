#include <opencv2/video/tracking.hpp>

#include "Tracker.h"

namespace cmt {

void Tracker::track(const GpuMat im_prev, const GpuMat im_gray, const vector<Point2f> & points_prev,
        vector<Point2f> & points_tracked, vector<unsigned char> & status)
{

    if (points_prev.size() > 0)
    {
        vector<float> err; //Needs to be float
        /* Convert everything to GpuMat */
        GpuMat points_prev_gpu(points_prev),points_tracked_gpu(points_tracked),status(status_gpu),err(err_gpu);
        //Calculate forward optical flow for prev_location
        cuda_of->calc(im_prev, im_gray, points_prev_gpu, points_tracked_gpu, status_gpu, err_gpu);
        
        /* Download useful stuff back to CPU */
        

        vector<Point2f> points_back;
        vector<unsigned char> status_back;
        vector<float> err_back; //Needs to be float

        //Calculate backward optical flow for prev_location
        cuda_of->calc(im_gray, im_prev, points_tracked, points_back, status_back, err_back);

        // TODO: download useful stuff back

        //Traverse vector backward so we can remove points on the fly
        for (int i = points_prev.size()-1; i >= 0; i--)
        {
            float l2norm = norm(points_back[i] - points_prev[i]);

            bool fb_err_is_large = l2norm > thr_fb;

            if (fb_err_is_large || !status[i] || !status_back[i])
            {
                points_tracked.erase(points_tracked.begin() + i);

                //Make sure the status flag is set to 0
                status[i] = 0;
            }

        }

    }
}

} /* namespace cmt */
