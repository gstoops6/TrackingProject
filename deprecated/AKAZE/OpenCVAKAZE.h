#ifndef OPENCV_AKAZE
#define OPENCV_AKAZE
#endif

#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
inline constexpr double inlier_threshold = 2.0;
inline constexpr double nn_match_ratio = 0.75;
inline constexpr double ransac_threshold = 3.0;
inline constexpr int knn_best_matches = 2;
inline constexpr int required_matches = 10;
inline constexpr int valid_inliers = 8;
inline constexpr double inlier_rejection_ratio = 0.25;

extern "C"{
#endif


typedef struct AkazeTracker AkazeTracker;



typedef enum{
    Accepted,
    EmptyInlierMask,
    EmptyHomography,
    InsufficientMatches,
    InsufficientValidInlier,
    InsufficientRejectionRatio,
    IncorrectBounds,
    BoxConcave
} BoxRejectionCriteria;

typedef struct{
    size_t height;
    size_t width;
    size_t stride;
    void *data;
} FrameDescriptor;

typedef struct {
    float x;
    float y;
}bbPoint;

typedef struct{
    bbPoint ul;
    bbPoint ur;
    bbPoint ll;
    bbPoint lr;

    bbPoint centroid;

    bool valid;
    BoxRejectionCriteria crit;
    double rejection_value;
}BoundingBox;


AkazeTracker *akaze_init(const FrameDescriptor *first_frame, const BoundingBox *initial_box);
BoundingBox akaze_compute(AkazeTracker *tracker, FrameDescriptor *frame);
void akaze_destroy(AkazeTracker *tracker);

#ifdef __cplusplus
}
#endif
