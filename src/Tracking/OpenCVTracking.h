#ifndef OPENCV_TRACKING
#define OPENCV_TRACKING
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
typedef struct CamshiftTracker CamshiftTracker;
typedef struct KCFTracker KCFTracker;

// Enhanced Camshift Tracker
typedef struct {
    AkazeTracker *ak;
    CamshiftTracker *cs;
    KCFTracker *kcf;
}ECT;

typedef enum{
    Accepted,
    EmptyInlierMask,
    EmptyHomography,
    InsufficientMatches,
    InsufficientValidInlier,
    InsufficientRejectionRatio,
    IncorrectBounds,
    BoxConcave,
    BhattacharyyaInsufficient,
    CamshiftHeightRatio,
    CamshiftWidthRatio,
    ExceedsWindow,
    CenterDisplacementExcessive,
    OverlappedAreaInsufficient,
    KCFInvalid
} BoxRejectionCriteria;


typedef enum{
    AKAZE,
    CAMSHIFT,
    IMM,
    KCF
} MatchingAlgorithm;

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
    MatchingAlgorithm matcher;
    BoxRejectionCriteria crit;
    double rejection_value;
}BoundingBox;



CamshiftTracker *camshift_init(const FrameDescriptor *initial_frame, const BoundingBox *bb, const float initial_matches);
BoundingBox camshift_compute(CamshiftTracker *tracker, const FrameDescriptor *frame, const BoundingBox *search_box);
void camshift_destroy(CamshiftTracker *tracker);

AkazeTracker *akaze_init(const FrameDescriptor *first_frame, const BoundingBox *bb, float *initial_matches);
BoundingBox akaze_compute(AkazeTracker *tracker, FrameDescriptor *frame);
void akaze_destroy(AkazeTracker *tracker);

KCFTracker *kcf_init(FrameDescriptor *initial_frame, BoundingBox *bb);
BoundingBox kcf_compute(KCFTracker *tracker, FrameDescriptor *frame, float *score);
void kcf_destroy(KCFTracker *t);

#ifdef __cplusplus
}
#endif

