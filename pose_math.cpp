#include "pose_math.hpp"

#include <opencv2/calib3d.hpp>

#include <array>
#include <cmath>
#include <stdexcept>

namespace umbra::pose {

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

cv::Mat makeT(cv::Mat R, cv::Mat t) {
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    if (R.cols == 3 && R.rows == 3) R.copyTo(T(cv::Rect(0, 0, 3, 3)));
    if (t.total() == 3) {
        cv::Mat t_col = t.reshape(1, 3);
        t_col.copyTo(T(cv::Rect(3, 0, 1, 3)));
    }
    return T;
}

cv::Mat quatToMatrix(double w, double x, double y, double z) {
    cv::Mat R(3, 3, CV_64F);
    R.at<double>(0,0) = 1-2*y*y-2*z*z; R.at<double>(0,1) = 2*x*y-2*z*w;     R.at<double>(0,2) = 2*x*z+2*y*w;
    R.at<double>(1,0) = 2*x*y+2*z*w;     R.at<double>(1,1) = 1-2*x*x-2*z*z; R.at<double>(1,2) = 2*y*z+2*x*w;
    R.at<double>(2,0) = 2*x*z-2*y*w; R.at<double>(2,1) = 2*y*z+2*x*w; R.at<double>(2,2) = 1-2*x*x-2*y*y;
    return R;
}

cv::Mat eulerToRotation(double r_deg, double p_deg, double y_deg) {
    if (!std::isfinite(r_deg) || !std::isfinite(p_deg) || !std::isfinite(y_deg))
        throw std::invalid_argument("nonfinite camera mounting angle");
    double r = r_deg*M_PI/180.0, p = p_deg*M_PI/180.0, y = y_deg*M_PI/180.0;
    cv::Mat Rx = (cv::Mat_<double>(3,3) << 1,0,0, 0,cos(r),-sin(r), 0,sin(r),cos(r));
    cv::Mat Ry = (cv::Mat_<double>(3,3) << cos(p),0,sin(p), 0,1,0, -sin(p),0,cos(p));
    cv::Mat Rz = (cv::Mat_<double>(3,3) << cos(y),-sin(y),0, sin(y),cos(y),0, 0,0,1);
    return Rz * Ry * Rx;
}

void appendTagPoints(const cv::Mat& fieldFromTag,
                     const std::vector<cv::Point2f>& observedCorners,
                     std::vector<cv::Point3d>& objectPoints,
                     std::vector<cv::Point2f>& imagePoints) {
    constexpr double s = tagSizeMeters / 2.0;
    static const std::array<cv::Point3d,4> tagCorners{{
        {0, -s, -s}, {0, s, -s}, {0, s, s}, {0, -s, s}
    }};

    for (const auto& localPt : tagCorners) {
        cv::Mat ptMat = (cv::Mat_<double>(4,1) << localPt.x, localPt.y, localPt.z, 1.0);
        cv::Mat fieldPtMat = fieldFromTag * ptMat;
        objectPoints.push_back(cv::Point3d(fieldPtMat.at<double>(0,0),
                                           fieldPtMat.at<double>(1,0),
                                           fieldPtMat.at<double>(2,0)));
    }
    for (const auto& point : observedCorners) imagePoints.push_back(point);
}

bool solveOpticalPose(const std::vector<cv::Point3d>& objectPoints,
                      const std::vector<cv::Point2f>& imagePoints,
                      const cv::Mat& cameraMatrix, const cv::Mat& distortion,
                      cv::Mat& rvec, cv::Mat& tvec) {
    if (objectPoints.size() < 4 || objectPoints.size() != imagePoints.size() ||
        cameraMatrix.rows != 3 || cameraMatrix.cols != 3 ||
        cameraMatrix.type() != CV_64F || !cv::checkRange(cameraMatrix) ||
        cameraMatrix.at<double>(0,0) <= 0 || cameraMatrix.at<double>(1,1) <= 0 ||
        (cameraMatrix.at<double>(2,2) != 1) ||
        (!distortion.empty() && (!cv::checkRange(distortion) ||
          (distortion.total() != 4 && distortion.total() != 5 &&
           distortion.total() != 8 && distortion.total() != 12 &&
           distortion.total() != 14)))) return false;
    for (const auto& p : objectPoints)
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return false;
    for (const auto& p : imagePoints)
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) return false;
    try {
        if (!cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distortion,
                          rvec, tvec, false, cv::SOLVEPNP_SQPNP)) return false;
        if (!cv::checkRange(rvec) || !cv::checkRange(tvec)) return false;
        cv::solvePnPRefineLM(objectPoints, imagePoints, cameraMatrix, distortion,
                             rvec, tvec);
        return cv::checkRange(rvec) && cv::checkRange(tvec);
    } catch (const cv::Exception&) {
        return false;
    }
}

static bool validRigidTransform(const cv::Mat& transform) {
    if (transform.rows != 4 || transform.cols != 4 ||
        transform.type() != CV_64F || !cv::checkRange(transform)) return false;
    for (int c = 0; c < 3; ++c)
        if (std::abs(transform.at<double>(3,c)) > 1e-8) return false;
    if (std::abs(transform.at<double>(3,3) - 1.0) > 1e-8) return false;
    const cv::Mat rotation = transform(cv::Rect(0, 0, 3, 3));
    return cv::norm(rotation.t() * rotation - cv::Mat::eye(3, 3, CV_64F)) < 1e-6 &&
           std::abs(cv::determinant(rotation) - 1.0) < 1e-6;
}

cv::Mat fieldFromRobotFromOptical(const cv::Mat& rvec, const cv::Mat& tvec,
                                 const cv::Mat& robotFromCamera) {
    cv::Mat R_f_c;
    cv::Rodrigues(rvec, R_f_c);
    cv::Mat T_c_f_ocv = makeT(R_f_c, tvec).inv();
    cv::Mat M_nwu_to_ocv = (cv::Mat_<double>(4,4) <<
        0,-1,0,0,  0,0,-1,0,  1,0,0,0,  0,0,0,1);
    // T_c_f_ocv maps optical coordinates into field NWU. Convert a camera
    // NWU vector to optical before applying it, then remove the camera mount.
    cv::Mat T_cam_to_field_nwu = T_c_f_ocv * M_nwu_to_ocv;
    return T_cam_to_field_nwu * robotFromCamera.inv();
}

Result robotPoseFromOptical(const cv::Mat& rvec, const cv::Mat& tvec,
                            const cv::Mat& robotFromCamera) {
    cv::Mat T_f_r = fieldFromRobotFromOptical(rvec, tvec, robotFromCamera);
    return {T_f_r.at<double>(0,3), T_f_r.at<double>(1,3),
            atan2(T_f_r.at<double>(1,0), T_f_r.at<double>(0,0)),
            cv::norm(tvec)};
}

bool solveRobotPose(const std::vector<cv::Point3d>& objectPoints,
                    const std::vector<cv::Point2f>& imagePoints,
                    int acceptedTags, const cv::Mat& cameraMatrix,
                    const cv::Mat& distortion, const cv::Mat& robotFromCamera,
                    Result& result) {
    if (acceptedTags <= 1) return false;
    cv::Mat rvec, tvec;
    if (!solveOpticalPose(objectPoints, imagePoints, cameraMatrix,
                          distortion, rvec, tvec)) return false;
    if (!validRigidTransform(robotFromCamera)) return false;
    try {
        const Result candidate = robotPoseFromOptical(rvec, tvec, robotFromCamera);
        if (!std::isfinite(candidate.x) || !std::isfinite(candidate.y) ||
            !std::isfinite(candidate.yaw) || !std::isfinite(candidate.distance)) return false;
        result = candidate;
        return true;
    } catch (const cv::Exception&) {
        return false;
    }
}

} // namespace umbra::pose
