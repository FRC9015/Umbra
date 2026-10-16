#include "pose_math.hpp"

#include <nlohmann/json.hpp>
#include <opencv2/calib3d.hpp>

#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

using Json = nlohmann::json;

namespace {

constexpr double pi = 3.14159265358979323846;
constexpr double tagHalf = 0.1651 / 2.0;

struct Rigid {
    cv::Matx33d rotation = cv::Matx33d::eye();
    cv::Vec3d translation{0, 0, 0};
};

cv::Matx33d aroundX(double angle) {
    const double c = std::cos(angle), s = std::sin(angle);
    return {1,0,0, 0,c,-s, 0,s,c};
}
cv::Matx33d aroundY(double angle) {
    const double c = std::cos(angle), s = std::sin(angle);
    return {c,0,s, 0,1,0, -s,0,c};
}
cv::Matx33d aroundZ(double angle) {
    const double c = std::cos(angle), s = std::sin(angle);
    return {c,-s,0, s,c,0, 0,0,1};
}
double radians(double degrees) { return degrees * pi / 180.0; }

Rigid compose(const Rigid& left, const Rigid& right) {
    return {left.rotation * right.rotation,
            left.rotation * right.translation + left.translation};
}
Rigid inverse(const Rigid& value) {
    const cv::Matx33d r = value.rotation.t();
    return {r, -(r * value.translation)};
}
cv::Vec3d apply(const Rigid& value, const cv::Vec3d& p) {
    return value.rotation * p + value.translation;
}
Rigid withAngles(cv::Vec3d center, double roll, double pitch, double yaw) {
    return {aroundZ(radians(yaw)) * aroundY(radians(pitch)) *
            aroundX(radians(roll)), center};
}
cv::Mat toMat(const Rigid& value) {
    cv::Mat result = cv::Mat::eye(4, 4, CV_64F);
    for (int r = 0; r < 3; ++r) {
        result.at<double>(r, 3) = value.translation[r];
        for (int c = 0; c < 3; ++c)
            result.at<double>(r, c) = value.rotation(r, c);
    }
    return result;
}
cv::Mat toMat(const cv::Matx33d& value) { return cv::Mat(value).clone(); }
// This independent reference maps NWU (forward, left, up) to optical
// (right, down, forward). These axis definitions come from WPILib and OpenCV.
const cv::Matx33d opticalFromNwu(0,-1,0, 0,0,-1, 1,0,0);
const std::array<cv::Vec3d,4> canonicalCorners{{
    {0,-tagHalf,-tagHalf}, {0,tagHalf,-tagHalf},
    {0,tagHalf,tagHalf}, {0,-tagHalf,tagHalf}
}};

struct Camera {
    double fx=920, fy=915, cx=640, cy=400;
    std::array<double,8> distortion{};
    cv::Mat matrix() const {
        return (cv::Mat_<double>(3,3) << fx,0,cx, 0,fy,cy, 0,0,1);
    }
    cv::Mat coefficients() const {
        cv::Mat result(8, 1, CV_64F);
        for (int i = 0; i < 8; ++i) result.at<double>(i) = distortion[i];
        return result;
    }
};

// Deliberately uses the pinhole/rational distortion equations, not
// production appendTagPoints(), undistortPoints(), or projectPoints().
cv::Point2f project(const cv::Vec3d& optical, const Camera& camera) {
    if (optical[2] <= 0) throw std::runtime_error("synthetic tag behind camera");
    const double x = optical[0] / optical[2], y = optical[1] / optical[2];
    const double r2 = x*x + y*y, r4 = r2*r2, r6 = r4*r2;
    const auto& k = camera.distortion;
    const double scale = (1 + k[0]*r2 + k[1]*r4 + k[4]*r6) /
                         (1 + k[5]*r2 + k[6]*r4 + k[7]*r6);
    const double xd = x*scale + 2*k[2]*x*y + k[3]*(r2 + 2*x*x);
    const double yd = y*scale + k[2]*(r2 + 2*y*y) + 2*k[3]*x*y;
    const cv::Point2f pixel(static_cast<float>(camera.fx*xd + camera.cx),
                             static_cast<float>(camera.fy*yd + camera.cy));
    if (pixel.x < 0 || pixel.x >= 1280 || pixel.y < 0 || pixel.y >= 800)
        throw std::runtime_error("synthetic corner outside 1280x800 image");
    return pixel;
}

struct Scene {
    Rigid fieldFromRobot;
    Rigid robotFromCamera;
    Camera camera;
    std::vector<cv::Point3d> worldPoints;
    std::vector<cv::Point2f> imagePoints;
    std::vector<cv::Point2f> observedPoints;
    int tags = 0;
};

Rigid opticalFromField(const Scene& scene) {
    const Rigid fieldFromCamera = compose(scene.fieldFromRobot, scene.robotFromCamera);
    return compose({opticalFromNwu, {0,0,0}}, inverse(fieldFromCamera));
}

// Centers are expressed in camera NWU. Each canonical tag faces the camera.
// Changing its roll rotates the printed tag around its own normal.
void addTag(Scene& scene, double forward, double left, double up,
            double tagRoll = 0) {
    const Rigid fieldFromCamera = compose(scene.fieldFromRobot, scene.robotFromCamera);
    const Rigid cameraFromTag{aroundZ(pi) * aroundX(radians(tagRoll)),
                              {forward,left,up}};
    const Rigid fieldFromTag = compose(fieldFromCamera, cameraFromTag);
    const Rigid optical = opticalFromField(scene);
    std::vector<cv::Point2f> observed;
    for (const auto& local : canonicalCorners) {
        const cv::Vec3d world = apply(fieldFromTag, local);
        const cv::Vec3d cameraPoint = apply(optical, world);
        observed.push_back(project(cameraPoint, scene.camera));
        scene.observedPoints.push_back(observed.back());
    }
    umbra::pose::appendTagPoints(toMat(fieldFromTag), observed,
                                 scene.worldPoints, scene.imagePoints);
    ++scene.tags;
}

Scene scene(Rigid robot, Rigid mount, Camera camera = {},
            bool rotated = false, bool edges = false) {
    Scene result{robot, mount, camera};
    const double lateral = edges ? 0.92 : 0.35;
    addTag(result, 2.5, lateral, 0.05, rotated ? 32 : 0);
    addTag(result, 3.2, lateral-0.63, -0.16, rotated ? -18 : 0);
    return result;
}

void near(const std::string& label, double actual, double expected,
          double tolerance) {
    const double error = std::abs(actual - expected);
    if (!std::isfinite(actual) || error > tolerance) {
        std::cerr << label << ": expected=" << expected << " actual=" << actual
                  << " abs_error=" << error << " tolerance=" << tolerance << '\n';
        throw std::runtime_error("assertion failed: " + label);
    }
}
void angleNear(const std::string& label, double actual, double expected,
               double toleranceDegrees) {
    const double error = std::remainder(actual-expected, 2*pi);
    if (!std::isfinite(actual) || std::abs(error) > radians(toleranceDegrees)) {
        std::cerr << label << ": expected_deg=" << expected*180/pi
                  << " actual_deg=" << actual*180/pi
                  << " wrapped_error_deg=" << error*180/pi
                  << " tolerance_deg=" << toleranceDegrees << '\n';
        throw std::runtime_error("assertion failed: " + label);
    }
}
void expectRobot(const umbra::pose::Result& actual, const Rigid& expected,
                 double positionTolerance = 0.003, double yawTolerance = 0.2) {
    int failures = 0;
    try { near("robot x (m)", actual.x, expected.translation[0], positionTolerance); }
    catch (const std::exception&) { ++failures; }
    try { near("robot y (m)", actual.y, expected.translation[1], positionTolerance); }
    catch (const std::exception&) { ++failures; }
    try {
        angleNear("robot yaw", actual.yaw,
                  std::atan2(expected.rotation(1,0), expected.rotation(0,0)),
                  yawTolerance);
    } catch (const std::exception&) { ++failures; }
    if (failures) throw std::runtime_error(std::to_string(failures) + " robot pose components failed");
}

void expectOptical(const Scene& sample, double tolerance = 0.002) {
    cv::Mat rvec, tvec;
    if (!umbra::pose::solveOpticalPose(sample.worldPoints, sample.imagePoints,
                                       sample.camera.matrix(), sample.camera.coefficients(),
                                       rvec, tvec))
        throw std::runtime_error("solveOpticalPose returned false");
    cv::Mat solvedRotation;
    cv::Rodrigues(rvec, solvedRotation);
    const Rigid expected = opticalFromField(sample);
    for (int row=0; row<3; ++row) {
        near("optical translation component " + std::to_string(row),
             tvec.at<double>(row), expected.translation[row], tolerance);
        for (int col=0; col<3; ++col)
            near("optical rotation component " + std::to_string(row) + "," +
                 std::to_string(col), solvedRotation.at<double>(row,col),
                 expected.rotation(row,col), tolerance);
    }
}

void expectRecovered(const Scene& sample) {
    umbra::pose::Result result{};
    if (!umbra::pose::solveRobotPose(sample.worldPoints, sample.imagePoints,
                                     sample.tags, sample.camera.matrix(),
                                     sample.camera.coefficients(),
                                     toMat(sample.robotFromCamera), result))
        throw std::runtime_error("solveRobotPose returned false");
    expectRobot(result, sample.fieldFromRobot);
}

void expectBasis(const Rigid& robot, const Rigid& mount) {
    const Scene sample{robot, mount, {}};
    const Rigid optical = opticalFromField(sample);
    cv::Mat rvec;
    cv::Rodrigues(toMat(optical.rotation), rvec);
    const cv::Mat actual = umbra::pose::fieldFromRobotFromOptical(
        rvec, (cv::Mat_<double>(3,1) << optical.translation[0],
                                         optical.translation[1],
                                         optical.translation[2]), toMat(mount));
    const cv::Mat expected = toMat(robot);
    for (int r=0; r<4; ++r) for (int c=0; c<4; ++c)
        near("field-from-robot matrix " + std::to_string(r) + "," +
             std::to_string(c), actual.at<double>(r,c),
             expected.at<double>(r,c), 1e-10);
}

Json loadJson(const std::string& name) {
    std::ifstream input(std::string(UMBRA_SOURCE_DIR) + "/" + name);
    if (!input) throw std::runtime_error("cannot open " + name);
    return Json::parse(input);
}

Rigid configuredMount(const std::string& name, Camera& camera) {
    const Json cfg = loadJson("config.json");
    for (const auto& entry : cfg.at("cameras")) {
        if (entry.at("name") != name) continue;
        const auto& offset = entry.at("offset");
        const auto& calib = entry.at("calibration");
        for (const char* field : {"x", "y", "z", "roll", "pitch", "yaw"}) {
            if (!std::isfinite(offset.at(field).get<double>()))
                throw std::runtime_error(name + ": nonfinite " + field);
        }
        for (const char* field : {"fx", "fy", "cx", "cy"}) {
            if (!std::isfinite(calib.at(field).get<double>()))
                throw std::runtime_error(name + ": nonfinite calibration " + field);
        }
        camera.fx = calib.at("fx"); camera.fy = calib.at("fy");
        camera.cx = calib.at("cx"); camera.cy = calib.at("cy");
        if (camera.fx <= 0 || camera.fy <= 0)
            throw std::runtime_error(name + ": invalid focal length");
        const auto dist = calib.at("dist").get<std::vector<double>>();
        if (dist.size() != 8) throw std::runtime_error(name + ": expected eight coefficients");
        for (size_t i=0; i<8; ++i) {
            if (!std::isfinite(dist[i])) throw std::runtime_error(name + ": nonfinite distortion");
            camera.distortion[i] = dist[i];
        }
        const Rigid mount = withAngles({offset.at("x").get<double>(),
                                        offset.at("y").get<double>(),
                                        offset.at("z").get<double>()},
                                       offset.at("roll"), offset.at("pitch"),
                                       offset.at("yaw"));
        const cv::Matx33d identity = mount.rotation * mount.rotation.t();
        for (int i=0; i<3; ++i)
            for (int j=0; j<3; ++j)
                near("mount orthogonality", identity(i,j), i==j ? 1.0 : 0.0, 1e-10);
        near("mount determinant", cv::determinant(toMat(mount.rotation)), 1.0, 1e-10);
        return mount;
    }
    throw std::runtime_error("configured camera missing: " + name);
}

Rigid standardRobot() { return withAngles({4.1,3.2,0.13}, 0,0,0); }

void run(const std::string& name) {
    const Rigid robot = standardRobot();
    if (name.starts_with("basis.")) {
        if (name == "basis.identity") expectBasis(withAngles({4.1,3.2,0.13},0,0,0), {});
        if (name == "basis.axes") expectBasis(withAngles({4.1,3.2,0.13},0,0,90), {});
        if (name == "basis.mount") expectBasis(withAngles({4.1,3.2,0.13},5,-8,35),
                                                withAngles({-0.23,-0.38,0.2},3,15,270));
    } else if (name == "math.euler") {
        cv::Mat actual = umbra::pose::eulerToRotation(12,-18,37);
        const auto expected = withAngles({0,0,0}, 12,-18,37).rotation;
        for (int r=0; r<3; ++r) for (int c=0; c<3; ++c)
            near("Euler rotation", actual.at<double>(r,c), expected(r,c), 1e-12);
    } else if (name == "math.quaternion") {
        cv::Mat actual = umbra::pose::quatToMatrix(std::cos(pi/4),0,0,std::sin(pi/4));
        const auto expected = aroundZ(pi/2);
        for (int r=0; r<3; ++r) for (int c=0; c<3; ++c)
            near("quaternion rotation", actual.at<double>(r,c), expected(r,c), 1e-12);
    } else if (name == "math.corner_order") {
        const Scene sample = scene(robot, withAngles({0,0,0},0,0,0));
        for (int i=0; i<4; ++i) {
            const Rigid fieldFromCamera = compose(sample.fieldFromRobot, sample.robotFromCamera);
            const Rigid fieldFromTag = compose(fieldFromCamera,
                {aroundZ(pi), {2.5,0.35,0.05}});
            const cv::Vec3d expected = apply(fieldFromTag, canonicalCorners[i]);
            const cv::Point3d actual = sample.worldPoints[i];
            near("tag corner x", actual.x, expected[0], 1e-12);
            near("tag corner y", actual.y, expected[1], 1e-12);
            near("tag corner z", actual.z, expected[2], 1e-12);
        }
    } else if (name.starts_with("fixed.")) {
        Rigid mount{};
        if (name == "fixed.mount_forward") mount = withAngles({0.28,0,0},0,0,0);
        if (name == "fixed.mount_rear") mount = withAngles({-0.28,0,0},0,0,0);
        if (name == "fixed.mount_left") mount = withAngles({0,0.38,0},0,0,0);
        if (name == "fixed.mount_right") mount = withAngles({0,-0.38,0},0,0,0);
        if (name == "fixed.mount_height") mount = withAngles({0,0,0.24},0,0,0);
        if (name == "fixed.mount_rotated") mount = withAngles({-0.23,-0.38,0.2},0,15,270);
        if (name == "fixed.updated_starboard")
            mount = withAngles({-0.2301748,-0.3818636,0.1988566},0,15,270);
        if (name == "fixed.updated_port")
            mount = withAngles({-0.2301748,0.3824478,0.2369566},0,15,90);
        expectRecovered(scene(robot, mount));
    } else if (name.starts_with("yaw.")) {
        Rigid pose = robot, mount = withAngles({0.21,-0.32,0.19},0,0,35);
        if (name == "yaw.positive90") pose = withAngles({4.1,3.2,0.13},0,0,90);
        if (name == "yaw.negative90") pose = withAngles({4.1,3.2,0.13},0,0,-90);
        if (name == "yaw.180") pose = withAngles({4.1,3.2,0.13},0,0,180);
        if (name == "yaw.270") pose = withAngles({4.1,3.2,0.13},0,0,270);
        if (name == "yaw.wrap") pose = withAngles({4.1,3.2,0.13},0,0,179.5);
        if (name == "yaw.tilt") pose = withAngles({4.1,3.2,0.13},10,-7,0);
        if (name == "yaw.combined") {
            pose = withAngles({4.1,3.2,0.13},10,-7,124);
            mount = withAngles({0.2,-0.32,0.24},5,15,270);
        }
        expectRecovered(scene(pose, mount));
    } else if (name.starts_with("optical.")) {
        Camera camera;
        if (name == "optical.radial") camera.distortion = {0.14,-0.05,0,0,0.005,0,0,0};
        if (name == "optical.tangential") camera.distortion = {0,0,0.012,-0.009,0,0,0,0};
        if (name == "optical.edge") camera.distortion = {0.14,-0.05,0.012,-0.009,0.005,0,0,0};
        if (name == "optical.alternate_intrinsics") {
            camera.fx = 735; camera.fy = 764; camera.cx = 610; camera.cy = 412;
        }
        expectOptical(scene(robot, {}, camera, false, name == "optical.edge"));
    } else if (name == "tag.single") {
        Scene sample{robot, {}, {}};
        addTag(sample, 2.5, 0.25, 0);
        umbra::pose::Result result{7,8,9,10};
        if (umbra::pose::solveRobotPose(sample.worldPoints, sample.imagePoints, 1,
                                       sample.camera.matrix(), sample.camera.coefficients(),
                                       toMat(sample.robotFromCamera), result))
            throw std::runtime_error("single-tag policy unexpectedly accepted");
        near("unchanged result on rejection", result.x, 7, 0);
    } else if (name == "tag.multi" || name == "tag.rotated" || name == "tag.planar") {
        Scene sample{robot, {}, {}};
        addTag(sample, 2.5, 0.35, 0.05, name == "tag.rotated" ? 32 : 0);
        addTag(sample, name == "tag.planar" ? 2.5 : 3.2,
               -0.28, name == "tag.planar" ? 0.05 : -0.16,
               name == "tag.rotated" ? -18 : 0);
        expectOptical(sample, 0.003);
    } else if (name == "tag.distance") {
        Scene nearSample = scene(robot, {});
        Scene farSample{robot, {}, {}};
        addTag(farSample, 5.2, 0.35, 0.05);
        addTag(farSample, 6.4, -0.28, -0.16);
        expectOptical(nearSample, 0.003);
        expectOptical(farSample, 0.003);
    } else if (name == "range.tag_distance") {
        Scene sample = scene(robot, {});
        umbra::pose::Result result{};
        if (!umbra::pose::solveRobotPose(sample.worldPoints, sample.imagePoints,
                sample.tags, sample.camera.matrix(), sample.camera.coefficients(),
                toMat(sample.robotFromCamera), result))
            throw std::runtime_error("pose solve failed");
        const double nearestTagDistance = std::sqrt(2.5*2.5 + 0.35*0.35 + 0.05*0.05);
        near("distance to nearest tag (m)", result.distance, nearestTagDistance, 0.003);
    } else if (name.starts_with("config.")) {
        Camera camera;
        const Rigid mount = configuredMount(name.substr(7), camera);
        expectRecovered(scene(withAngles({4.1,3.2,0.13},0,0,25), mount, camera));
    } else if (name == "invalid.nonfinite_mount") {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        bool rejected = false;
        try { (void)umbra::pose::eulerToRotation(0,nan,0); }
        catch (const std::invalid_argument&) { rejected = true; }
        if (!rejected) throw std::runtime_error("nonfinite mount accepted");
    } else if (name == "invalid.degenerate_points") {
        const Camera camera;
        const std::vector<cv::Point3d> world(8, {2,0,1});
        const std::vector<cv::Point2f> pixels(8, {640,400});
        umbra::pose::Result result{};
        if (umbra::pose::solveRobotPose(world, pixels, 2, camera.matrix(),
                camera.coefficients(), toMat(Rigid{}), result))
            throw std::runtime_error("degenerate geometry accepted");
    } else if (name == "invalid.bad_intrinsics") {
        Scene sample = scene(robot, {});
        cv::Mat bad = sample.camera.matrix();
        bad.at<double>(0,0) = 0;
        umbra::pose::Result result{};
        if (umbra::pose::solveRobotPose(sample.worldPoints, sample.imagePoints,
                sample.tags, bad, sample.camera.coefficients(), toMat(Rigid{}), result))
            throw std::runtime_error("zero focal length accepted");
    } else if (name == "invalid.singular_mount") {
        Scene sample = scene(robot, {});
        umbra::pose::Result result{};
        if (umbra::pose::solveRobotPose(sample.worldPoints, sample.imagePoints,
                sample.tags, sample.camera.matrix(), sample.camera.coefficients(),
                cv::Mat::zeros(4,4,CV_64F), result))
            throw std::runtime_error("singular robot-from-camera accepted");
    } else if (name == "invalid.malformed_config") {
        bool rejected = false;
        try { const auto parsed = Json::parse("{\"cameras\": [}"); (void)parsed; }
        catch (const Json::parse_error&) { rejected = true; }
        if (!rejected) throw std::runtime_error("malformed JSON accepted by parser");
    } else if (name == "field.reference") {
        const Json field = loadJson("field.json");
        near("field length (m)", field.at("field").at("length"), 16.541, 1e-9);
        near("field width (m)", field.at("field").at("width"), 8.069, 1e-9);
        const auto& tags = field.at("tags");
        if (tags.size() != 32) throw std::runtime_error("expected 32 field tags");
        const auto& first = tags.at(0);
        if (first.at("ID") != 1) throw std::runtime_error("field tag 1 missing");
        near("tag 1 x", first.at("pose").at("translation").at("x"), 11.8779798, 1e-9);
    } else if (name == "benchmark.synthetic_pose") {
        // Host-only PnP microbenchmark; no camera, decoder, NetworkTables,
        // frame timestamp, or ARM64 CPU is involved.
        const Scene sample = scene(robot, {});
        const cv::Mat intrinsics = sample.camera.matrix();
        const cv::Mat distortion = sample.camera.coefficients();
        std::vector<double> durations;
        durations.reserve(300);
        for (int i=0; i<320; ++i) {
            const auto start = std::chrono::steady_clock::now();
            cv::Mat rvec, tvec;
            if (!umbra::pose::solveOpticalPose(sample.worldPoints, sample.imagePoints,
                    intrinsics, distortion, rvec, tvec))
                throw std::runtime_error("benchmark solve failed");
            const auto stop = std::chrono::steady_clock::now();
            if (i >= 20) durations.push_back(std::chrono::duration<double, std::milli>(stop-start).count());
        }
        std::sort(durations.begin(), durations.end());
        const double total = std::accumulate(durations.begin(), durations.end(), 0.0);
        std::cout << "host_synthetic_pnp_ms median=" << durations[durations.size()/2]
                  << " p95=" << durations[static_cast<size_t>(durations.size()*0.95)]
                  << " mean=" << total/durations.size() << " samples=" << durations.size()
                  << '\n';
    } else {
        throw std::runtime_error("unknown case: " + name);
    }
}

} // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: umbra_pose_tests CASE_NAME\n";
        return 2;
    }
    try {
        run(argv[1]);
        std::cout << "PASS " << argv[1] << '\n';
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "FAIL " << argv[1] << ": " << e.what() << '\n';
        return 1;
    }
}
