#include "OpenCVTracking.h"
#include <iostream>
#include <math.h>
#include "opencv2/core/hal/interface.h"
#include "opencv2/geometry/2d.hpp"
#include "opencv2/tracking.hpp"
#include "opencv2/core.hpp"
#include "opencv2/geometry.hpp"
#include "opencv2/geometry/3d.hpp"
#include "opencv2/core/cvdef.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/video.hpp"

using namespace cv;
using namespace std;

struct KCFTracker{
    Ptr<TrackerKCF> tracker;
    TrackerKCF::Params params;
};


inline bbPoint point2f_to_struct(Point2f p){
    return {
        .x = p.x,
        .y = p.y
    };
}


inline BoundingBox points_to_box(
    Point2f v[],
    bool valid = true,
    bbPoint centroid = {.x = 0, .y = 0},
    BoxRejectionCriteria crit = Accepted,
    double rej_val = 0
    ){

    return {
        .ul = point2f_to_struct(v[1]),
        .ur = point2f_to_struct(v[2]),
        .ll = point2f_to_struct(v[0]),
        .lr = point2f_to_struct(v[3]),
        .centroid = centroid,
        .valid = valid,
        .matcher = KCF,
        .crit = crit,

        .rejection_value = rej_val
    };
}


Rect bounding_box_to_cvRect(const BoundingBox *bb){
    return boundingRect(vector<Point2f>{
        Point2f(bb->ul.x, bb->ul.y),
        Point2f(bb->ur.x, bb->ur.y),
        Point2f(bb->lr.x, bb->lr.y),
        Point2f(bb->ll.x, bb->ll.y)
    });
}




// Pass full initial frame
KCFTracker *kcf_init(FrameDescriptor *initial_frame, BoundingBox *bb){
    Ptr<TrackerKCF> objectTracker;

    auto params = TrackerKCF::Params();
    //params.desc_npca = TrackerKCF::GRAY;
    params.desc_npca = TrackerKCF::CN;

    //params.compress_feature = true;
    //params.compressed_size = 2;
    //params.resize = true;
    //params.wrap_kernel = false;
    //params.pca_learning_rate = 0.012f;

    objectTracker = TrackerKCF::create(params);
    Rect bb_rect = bounding_box_to_cvRect(bb);

    Mat bgr_frame;
    Mat rgb_frame(initial_frame->height, initial_frame->width, CV_8UC3, initial_frame->data, initial_frame->stride);

    cvtColor(rgb_frame, bgr_frame, COLOR_RGB2BGR);
    objectTracker->init(bgr_frame, bb_rect);

    Rect updated_bb;
    auto result = objectTracker->update(bgr_frame, updated_bb);

    return new KCFTracker{
        .tracker = objectTracker,
        .params = params
    };
}


BoundingBox kcf_compute(KCFTracker *tracker, FrameDescriptor *frame, float *score){
    Mat rgb_frame(frame->height, frame->width, CV_8UC3, frame->data, frame->stride);
    Mat bgr_frame;
    cvtColor(rgb_frame, bgr_frame, COLOR_RGB2BGR);

    Rect bb_rect;
    try{
        auto result = tracker->tracker->update(bgr_frame,bb_rect);
        const auto x = static_cast<float>(bb_rect.x);
        const auto y = static_cast<float>(bb_rect.y);
        const auto width = static_cast<float>(bb_rect.width);
        const auto height = static_cast<float>(bb_rect.height);


        Point2f points[4] = {
            Point2f(x, y + height),
            Point2f(x, y),
            Point2f(x + width, y),
            Point2f(x + width, y + height)
        };

        const BoxRejectionCriteria br = result ? Accepted : KCFInvalid;
        *score = tracker->tracker->getTrackingScore();
        return points_to_box(points, result, {x + width / 2.0f, y + height / 2.0f}, br);

    } catch (const exception& e){
        return {.valid = false};
    }
}


void kcf_destroy(KCFTracker *t){
    delete t;
}
