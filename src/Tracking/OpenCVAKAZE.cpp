#include "OpenCVTracking.h"
#include <stddef.h>
#include <stdbool.h>
#include "opencv2/geometry/3d.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/core/ocl.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/geometry.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/features.hpp"
#include <cstddef>
#include <ctime>
#include <iostream>
#include <vector>



using namespace std;
using namespace cv;


struct AkazeTracker {
    Ptr<xfeatures2d::AKAZE> detector;
    vector<KeyPoint> reference_keypoints;
    UMat reference_descriptors;
    vector<Point2f> initial_box;
};


inline bbPoint point2f_to_struct(Point2f p){
    return {
        .x = p.x,
        .y = p.y
    };
}

inline BoundingBox points_to_box(vector<Point2f> &v, bool valid = true, bbPoint centroid = {0, 0}, BoxRejectionCriteria crit = Accepted, double rej_val = 0){
    if(v.size() != 4) return {.valid = false};

    return{
        .ul = point2f_to_struct(v[0]),
        .ur = point2f_to_struct(v[1]),
        .ll = point2f_to_struct(v[3]),
        .lr = point2f_to_struct(v[2]),
        .centroid = centroid,
        .valid = valid,
        .matcher = AKAZE,
        .crit = crit,
        .rejection_value = rej_val
    };
}

inline vector<Point2f> box_to_points(const BoundingBox *bb){
    return {
        Point2f(cvRound(bb->ul.x), cvRound(bb->ul.y)),
        Point2f(cvRound(bb->ur.x), cvRound(bb->ur.y)),
        Point2f(cvRound(bb->lr.x), cvRound(bb->lr.y)),
        Point2f(cvRound(bb->ll.x), cvRound(bb->ll.y))
    };
}


inline bbPoint calculate_centroid(const vector<Point2f> &b, bool *convex){
    *convex = b.size() == 4 && isContourConvex(b);
    if(!*convex){
        return {.x = 0.0, .y = 0.0};
    }

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
        *convex = false;
        return {.x = 0.0, .y = 0.0};
    }
    cx /= (6 * area);
    cy /= (6 * area);

    return {.x = cx, .y = cy};
}




AkazeTracker *akaze_init(const FrameDescriptor *first_frame, const BoundingBox *bb, float *initial_matches){
    if(ocl::haveOpenCL()){
        ocl::setUseOpenCL(true);
    }
    bool stat = ocl::useOpenCL();
    std::string ocl_stat = stat ? "true" : "false";

    cout << "Using OpenCL: " << ocl_stat << endl;

    auto detector = xfeatures2d::AKAZE::create(
        xfeatures2d::AKAZE::DESCRIPTOR_KAZE,
        0,
        3,
        0.005f,
        4,
        4,
        xfeatures2d::KAZE::DIFF_PM_G2,
        -1
    );
    auto initial_box = box_to_points(bb);

    Mat color_frame(first_frame->height, first_frame->width, CV_8UC3, first_frame->data, first_frame->stride);
    UMat color_img, img;
    color_frame.copyTo(color_img);
    cvtColor(color_img, img, COLOR_RGB2GRAY);

    UMat reference_mask(img.size(), CV_8UC1, Scalar(0));

    vector<Point> mask_points;
    mask_points.reserve(initial_box.size());

    for (const auto &point : initial_box) {
        mask_points.emplace_back(cvRound(point.x), cvRound(point.y));
    }

    fillConvexPoly(reference_mask, mask_points, Scalar(255));

    vector<KeyPoint> kpts;
    UMat desc;
    detector->detectAndCompute(img, reference_mask, kpts, desc);

    *initial_matches = static_cast<float>(kpts.size());

    return new AkazeTracker {
        .detector = detector,
        .reference_keypoints = kpts,
        .reference_descriptors = desc,
        .initial_box = initial_box,
    };
}

BoundingBox akaze_compute(AkazeTracker *tracker, FrameDescriptor *frame){

    Mat color_frame(frame->height, frame->width, CV_8UC3, frame->data, frame->stride);
    UMat color_img, img;
    color_frame.copyTo(color_img);
    cvtColor(color_img, img, COLOR_RGB2GRAY);

    UMat desc;
    vector<KeyPoint> kpts;
    tracker->detector->detectAndCompute(img, noArray(), kpts, desc);

    BFMatcher matcher(NORM_L2);
    vector< vector<DMatch> > nn_matches;
    matcher.knnMatch(desc, tracker->reference_descriptors, nn_matches, knn_best_matches);

    vector<KeyPoint> matched1, matched2;
    vector<Point2f> obj, scene;
    for(size_t i = 0; i < nn_matches.size(); i++) {
        DMatch first = nn_matches[i][0];
        float dist1 = nn_matches[i][0].distance;
        float dist2 = nn_matches[i][1].distance;

        if(dist1 < nn_match_ratio * dist2) {
            auto kp1 = kpts[first.queryIdx];
            auto kp2 = tracker->reference_keypoints[first.trainIdx];
            matched1.push_back(kp1);
            //matched2.push_back(kp2);

            obj.push_back(kp1.pt);
            scene.push_back(kp2.pt);
        }
    }


    bool valid_frame = true;
    Mat homography, inlier_mask;
    if(scene.size() >= required_matches){
        homography = findHomography(
            scene,
            obj,
            RANSAC,
            ransac_threshold,
            inlier_mask
        );

        if(inlier_mask.empty()) return {.valid = false, .crit = EmptyInlierMask};


        if(homography.empty()) return {.valid = false, .crit = EmptyHomography};

        const size_t non_zero_points = countNonZero(inlier_mask);
        const double ratio = static_cast<double>(non_zero_points) / scene.size();

        if(non_zero_points < valid_inliers) return {.valid = false, .crit = InsufficientValidInlier, .rejection_value = static_cast<double>(non_zero_points)};
        if(ratio < inlier_rejection_ratio) return {.valid = false, .crit = InsufficientRejectionRatio, .rejection_value = ratio};

    }else {
        return {.valid = false, .crit = InsufficientMatches, .rejection_value =  static_cast<double>(scene.size())};
    }

    vector<Point2f> new_bb;
    
    perspectiveTransform(tracker->initial_box, new_bb, homography);
    BoxRejectionCriteria brc = Accepted;

    for(auto &p : new_bb){
        float px = p.x;
        float py = p.y;

        if((px < 0 || px > frame->width) || (py < 0 || py > frame->height)){
            valid_frame = false;
            brc = IncorrectBounds;
            break;
        }
    }

    bool convex;
    auto center = calculate_centroid(new_bb, &convex);
    valid_frame = valid_frame && convex;

    if(!valid_frame && brc == Accepted){
        brc = BoxConcave;
    }
    if(valid_frame){
        tracker->initial_box = new_bb;
        tracker->reference_descriptors = desc;
        tracker->reference_keypoints = kpts;
    }

    return points_to_box(new_bb, valid_frame, center, brc);
}

void akaze_destroy(AkazeTracker *tracker){
    delete tracker;
}

