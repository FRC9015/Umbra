#pragma once

#include <opencv2/core.hpp>

#include <vector>

namespace umbra::pose {

inline constexpr double tagSizeMeters = 0.1651;

// These functions contain the existing production mathematics. Stage 1 tests
// intentionally exercise the current behavior, including known defects.
cv::Mat makeT(cv::Mat rotation, cv::Mat translation);
cv::Mat quatToMatrix(double w, double x, double y, double z);
cv::Mat eulerToRotation(double rollDegrees, double pitchDegrees, double yawDegrees);

void appendTagPoints(const cv::Mat& fieldFromTag,
                     const std::vector<cv::Point2f>& observedCorners,
                     std::vector<cv::Point3d>& objectPoints,
                     std::vector<cv::Point2f>& imagePoints);

struct Result {
    double x;
    double y;
    double yaw;
    double distance;
};

// Both phases are also called by solveRobotPose. Exposing them permits a test
// to isolate image calibration from the coordinate-basis conversion.
bool solveOpticalPose(const std::vector<cv::Point3d>& objectPoints,
                      const std::vector<cv::Point2f>& imagePoints,
                      const cv::Mat& cameraMatrix, const cv::Mat& distortion,
                      cv::Mat& rvec, cv::Mat& tvec);
cv::Mat fieldFromRobotFromOptical(const cv::Mat& rvec, const cv::Mat& tvec,
                                 const cv::Mat& robotFromCamera);
Result robotPoseFromOptical(const cv::Mat& rvec, const cv::Mat& tvec,
                            const cv::Mat& robotFromCamera);

// A single tag is deliberately unsupported by the current production policy.
// A failed solve returns false and leaves result untouched.
bool solveRobotPose(const std::vector<cv::Point3d>& objectPoints,
                    const std::vector<cv::Point2f>& imagePoints,
                    int acceptedTags, const cv::Mat& cameraMatrix,
                    const cv::Mat& distortion, const cv::Mat& robotFromCamera,
                    Result& result);

} // namespace umbra::pose
