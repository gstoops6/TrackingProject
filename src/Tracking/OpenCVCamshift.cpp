#include "OpenCVTracking.h"
#include <iostream>
#include <math.h>
#include "opencv2/core.hpp"
#include "opencv2/geometry.hpp"
#include "opencv2/geometry/3d.hpp"
#include "opencv2/core/cvdef.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/video.hpp"


using namespace cv;
using namespace std;

const Scalar hsv_lower_bound = Scalar(0, 15, 20);
const Scalar hsv_upper_bound = Scalar(255, 255, 255);
const float bhattacharyya_threshold = 0.6f;
const double displacement_ratio = 0.2;

struct CamshiftTracker {
    Mat obj_hist;
    Mat scene_hist;
    Rect window;
    TermCriteria term;
    int channels[1];
    float ranges[2];
    int hist_size[1];
    float initial_matches;
};

inline vector<Point2f> box_to_points(const BoundingBox *bb){
    return {
        Point2f(cvRound(bb->ul.x), cvRound(bb->ul.y)),
        Point2f(cvRound(bb->ur.x), cvRound(bb->ur.y)),
        Point2f(cvRound(bb->lr.x), cvRound(bb->lr.y)),
        Point2f(cvRound(bb->ll.x), cvRound(bb->ll.y))
    };
}


inline bbPoint calculate_centroid_box(const BoundingBox *bb){
    auto b = box_to_points(bb);
    float area = 0.0;
    float cx = 0.0;
    float cy = 0.0;

    #pragma unroll 4
    for(int i = 0; i < 4; i++){
        int j = (i + 1) % 4;

        float diff = b[i].x * b[j].y - b[j].x * b[i].y;
        area += diff;
        cx += (b[i].x + b[j].x) * diff;
        cy += (b[i].y + b[j].y) * diff;
    }

    area *= 0.5;
    if(area == 0.0f){
        return {.x = 0.0, .y = 0.0};
    }
    cx /= (6 * area);
    cy /= (6 * area);

    return {.x = cx, .y = cy};
}


inline bbPoint calculate_centroid_points(Point2f b[]){

    float area = 0.0;
    float cx = 0.0;
    float cy = 0.0;

    #pragma unroll 4
    for(int i = 0; i < 4; i++){
        int j = (i + 1) % 4;

        float diff = b[i].x * b[j].y - b[j].x * b[i].y;
        area += diff;
        cx += (b[i].x + b[j].x) * diff;
        cy += (b[i].y + b[j].y) * diff;
    }

    area *= 0.5;
    if(area == 0.0f){
        return {.x = 0.0, .y = 0.0};
    }
    cx /= (6 * area);
    cy /= (6 * area);

    return {.x = cx, .y = cy};
}


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
    double rej_val = 0){
    return {
        .ul = point2f_to_struct(v[1]),
        .ur = point2f_to_struct(v[2]),
        .ll = point2f_to_struct(v[0]),
        .lr = point2f_to_struct(v[3]),
        .centroid = centroid,
        .valid = valid,
        .matcher = CAMSHIFT,
        .crit = crit,

        .rejection_value = rej_val
    };
}

Rect bounding_box_to_rect(const BoundingBox *bb){
    return boundingRect(vector<Point2f>{
        Point2f(bb->ul.x, bb->ul.y),
        Point2f(bb->ur.x, bb->ur.y),
        Point2f(bb->lr.x, bb->lr.y),
        Point2f(bb->ll.x, bb->ll.y)
    });
}


bool bhattacharyya(const Mat &obj, const Mat &scene, float threshold) {
    if(obj.empty() || scene.empty()) return false;
    CV_Assert(obj.type() == CV_32FC1);
    CV_Assert(scene.type() == CV_32FC1);
    CV_Assert(obj.size() == scene.size());

    if(cv::sum(obj)[0] <= 0 || cv::sum(scene)[0] <= 0) return false;

    const double d_bhat = compareHist(obj, scene, HISTCMP_BHATTACHARYYA);
    return d_bhat <= threshold;
}


CamshiftTracker *camshift_init(const FrameDescriptor *initial_frame, const BoundingBox *bb, const float initial_matches){

    Mat color_frame(initial_frame->height, initial_frame->width, CV_8UC3, initial_frame->data, initial_frame->stride);

    Mat hsv_roi, mask;
    const auto track_window = bounding_box_to_rect(bb) & Rect(0, 0, color_frame.cols, color_frame.rows);
    if(track_window.empty()) return nullptr;
    Mat roi = color_frame(track_window);

    cvtColor(roi, hsv_roi, COLOR_RGB2HSV);
    inRange(hsv_roi, hsv_lower_bound, hsv_upper_bound, mask);

    float range_[] = {0, 180};
    const float* range[] = {range_};
    Mat roi_hist;
    int histSize[] = {180};
    int channels[] = {0};
    calcHist(&hsv_roi, 1, channels, mask, roi_hist, 1, histSize, range);
    normalize(roi_hist, roi_hist, 0, 255, NORM_MINMAX);

    return new CamshiftTracker{
        .obj_hist = roi_hist,
        .window = track_window,
        .term = TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 10, 1),
        .channels = {0},
        .ranges = {0, 180},
        .hist_size = {180},
        .initial_matches = initial_matches
    };
}


bool validate_results(Point2f points[], const BoundingBox *box, size_t height, size_t width, BoxRejectionCriteria *br){
    const float h = static_cast<float>(height);
    const float w = static_cast<float>(width);
    for(int i = 0; i < 4; i++){
        auto point = points[i];
        if (point.x > w || point.x < 0 || point.y > h || point.y < 0) {
            *br = ExceedsWindow;
            return false;
        }
    }
    const Rect prior_rect = bounding_box_to_rect(box);
    const Rect current_rect = boundingRect(vector<Point2f>{
        points[0],
        points[1],
        points[2],
        points[3]
    });

    if(prior_rect.empty() || current_rect.empty()){
        *br = IncorrectBounds;
        return false;
    }

    const auto current_centroid = calculate_centroid_points(points);
    const double displacement = hypot(
        current_centroid.x - box->centroid.x,
        current_centroid.y - box->centroid.y
    );
    const double max_displacement = displacement_ratio * hypot(prior_rect.width, prior_rect.height);

    if(displacement > max_displacement){
        *br = CenterDisplacementExcessive;
        return false;
    }


    const float height_ratio = static_cast<float>(current_rect.height) / prior_rect.height;
    const float width_ratio = static_cast<float>(current_rect.width) / prior_rect.width;

    if(height_ratio > 2.0f || height_ratio < 0.5f){
        //cout << "Failing Height Ratio: " << height_ratio << endl;
        *br = CamshiftHeightRatio;
        return false;
    }
    if(width_ratio > 2.0f || width_ratio < 0.5f){
        //cout << "Failing Width Ratio: " << width_ratio << endl;
        *br = CamshiftWidthRatio;
        return false;
    }
    return true;
}


BoundingBox camshift_compute(CamshiftTracker *tracker, const FrameDescriptor *frame, const BoundingBox *search_box){
    Mat hsv, dst, mask;
    Mat rgb(frame->height, frame->width, CV_8UC3, frame->data, frame->stride);

    if(search_box == nullptr) return {.valid = false, .crit = IncorrectBounds};

    // search_box must be crop-relative when the supplied image is cropped.
    tracker->window = bounding_box_to_rect(search_box) & Rect(0, 0, rgb.cols, rgb.rows);
    if(tracker->window.empty()) return {.valid = false, .crit = IncorrectBounds};


    cvtColor(rgb, hsv, COLOR_RGB2HSV);
    inRange(hsv, hsv_lower_bound, hsv_upper_bound, mask);

    const float *ranges = tracker->ranges;
    Mat hsv_roi = hsv(tracker->window);
    Mat mask_roi = mask(tracker->window);
    calcHist(
        &hsv_roi,
         1,
         tracker->channels,
         mask_roi,
         tracker->scene_hist,
         1,
         tracker->hist_size,
         &ranges
    );

    if(!bhattacharyya(tracker->obj_hist, tracker->scene_hist, bhattacharyya_threshold)) return {.valid = false, .crit = BhattacharyyaInsufficient};

    calcBackProject(&hsv, 1, tracker->channels, tracker->obj_hist, dst, &ranges);
    dst &= mask;

    auto new_roi = CamShift(dst, tracker->window, tracker->term);

    Point2f points[4];
    new_roi.points(points);

    BoxRejectionCriteria br = Accepted;
    auto validate = validate_results(points, search_box, frame->height, frame->width, &br);

    return points_to_box(
    points,
    validate,
    calculate_centroid_points(points),
    br
    );
}


void camshift_destroy(CamshiftTracker *tracker){
    delete tracker;
}

/*
2.
Validate the resulting box. OpenCVCamshift.cpp:204 always returns valid = true. Check finite coordinates, positive dimensions, frame overlap, and excessive changes in area or position. Rejected results should trigger AKAZE.
3.
Check the final candidate’s histogram. The current similarity check evaluates the starting window. Camshift can subsequently drift into a different region, so that candidate also needs validation.
5.
Reassess hue-only tracking for this truck. White and black pixels contribute little under the current HSV mask. If color tracking remains weak after these fixes, a value-based model or greater reliance on AKAZE may work better.
*/
