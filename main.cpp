#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include <map>
#include <chrono>
#include <cmath>
#include <atomic>
#include <mutex>
#include <algorithm>

// Third-party Libraries
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <apriltag.h>
#include <tag36h11.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleArrayTopic.h>
#include "httplib.h"

using json = nlohmann::json;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

const double TAG_SIZE = 0.1651; // 6.5 inches
const std::string configPath = "/home/dietpi/Umbra/config.json";

std::mutex frameMutex;
std::map<std::string, cv::Mat> cameraFrames; 
std::atomic<bool> clientIsWatching{false};

int zarray_get_id(zarray_t* detections, int index) {
    apriltag_detection_t *det; zarray_get(detections, index, &det); return det->id;
}

struct CameraInstance {
    std::string name; std::string path;
    cv::Mat camMatrix; cv::Mat distCoeffs;
    cv::Mat T_robot_camera; nt::DoubleArrayPublisher pub;
    int exposure = 25; int gain = 0; int brightness = 0;
    bool auto_exposure = false;
};

struct FieldTag { 
    int id; cv::Mat T_field_tag; 
}; 
std::map<int, FieldTag> fieldMap;

// --- MATH HELPERS ---
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
    R.at<double>(1,0) = 2*x*y+2*z*w;     R.at<double>(1,1) = 1-2*x*x-2*z*z; R.at<double>(1,2) = 2*y*z-2*x*w;
    R.at<double>(2,0) = 2*x*z-2*y*w;     R.at<double>(2,1) = 2*y*z+2*x*w;     R.at<double>(2,2) = 1-2*x*x-2*y*y;
    return R;
}

cv::Mat eulerToRotation(double r_deg, double p_deg, double y_deg) {
    double r = r_deg*M_PI/180.0, p = p_deg*M_PI/180.0, y = y_deg*M_PI/180.0;
    cv::Mat Rx = (cv::Mat_<double>(3,3) << 1,0,0, 0,cos(r),-sin(r), 0,sin(r),cos(r));
    cv::Mat Ry = (cv::Mat_<double>(3,3) << cos(p),0,sin(p), 0,1,0, -sin(p),0,cos(p));
    cv::Mat Rz = (cv::Mat_<double>(3,3) << cos(y),-sin(y),0, sin(y),cos(y),0, 0,0,1);
    return Rz * Ry * Rx; 
}

void loadField(std::string path) {
    std::ifstream f(path); if (!f.is_open()) return;
    json data = json::parse(f);
    for (auto& tag : data["tags"]) {
        int id = tag["ID"];
        cv::Mat t = (cv::Mat_<double>(3,1) << (double)tag["pose"]["translation"]["x"], (double)tag["pose"]["translation"]["y"], (double)tag["pose"]["translation"]["z"]);
        auto q = tag["pose"]["rotation"]["quaternion"];
        cv::Mat R = quatToMatrix(q["W"], q["X"], q["Y"], q["Z"]);
        fieldMap[id] = {id, makeT(R, t)};
    }
}

// --- WEB SERVER (SPLIT-LAYOUT LEFT ALIGNED FOR LENS PARAMS) ---
void runWebServer(std::vector<CameraInstance*>& cameras) {
    httplib::Server svr;
    
    svr.Get("/", [&](const httplib::Request&, httplib::Response& res) {
        std::string html = R"html(
        <!DOCTYPE html>
        <html>
        <head>
            <meta charset='utf-8'>
            <title>Umbra Control Panel</title>
            <style>
                body { background: #121214; color: #e1e1e6; font-family: sans-serif; margin: 0; padding: 20px; text-align: left; }
                h1 { color: #8257e5; border-bottom: 2px solid #29292e; padding-bottom: 10px; margin-top: 0; margin-bottom: 25px; max-width: 1150px; }
                .camera-row { display: flex; flex-direction: row; gap: 24px; width: 100%; max-width: 1150px; margin-bottom: 30px; justify-content: flex-start; align-items: stretch; }
                .left-col { flex: 1.2; max-width: 640px; }
                .right-col { flex: 1; max-width: 480px; display: flex; }
                .card { background: #202024; border-radius: 8px; padding: 20px; border: 1px solid #29292e; width: 100%; box-sizing: border-box; display: flex; flex-direction: column; justify-content: space-between; }
                .stream-img { width: 100%; max-width: 600px; border-radius: 6px; border: 2px dashed #8257e5; background: #000; }
                h3 { margin-top: 0; color: #04d361; font-size: 18px; margin-bottom: 15px; }
                .row { display: flex; justify-content: space-between; align-items: center; margin-bottom: 12px; }
                .slider-wrap { display: flex; align-items: center; gap: 10px; }
                label { font-weight: bold; color: #a8a8b3; font-size: 14px; }
                input[type=range] { width: 150px; accent-color: #8257e5; }
                input[type=number] { background: #121214; border: 1px solid #29292e; color: #fff; padding: 6px; border-radius: 4px; width: 60px; text-align: center; }
                input[type=checkbox] { width: 20px; height: 20px; accent-color: #04d361; cursor: pointer; }
                button { background: #8257e5; border: none; color: white; padding: 10px; border-radius: 4px; cursor: pointer; font-weight: bold; width: 100%; transition: background 0.2s; margin-top: 15px; }
                button:hover { background: #9466ff; }
                .step-btn { padding: 4px 8px; background: #29292e; color: #e1e1e6; border: 1px solid #41414c; width: auto; display: inline-block; margin-right: 2px; margin-top: 0; }
            </style>
            <script>
                function update(cam, id, val) {
                    let slider = document.getElementById(cam + '_' + id + '_slider');
                    let num = document.getElementById(cam + '_' + id);
                    if (slider && id !== 'auto') slider.value = val;
                    if (num && id !== 'auto') num.value = val;
                    fetch('/set?cam=' + cam + '&' + id + '=' + val);
                }
                function changeValue(cam, id, step) {
                    let input = document.getElementById(cam + '_' + id);
                    let val = parseInt(input.value) + step;
                    update(cam, id, val);
                }
                function save(cam) { 
                    fetch('/save?cam=' + cam)
                        .then(r => r.text())
                        .then(txt => alert('Configurations saved to SD Card!')); 
                }
            </script>
        </head>
        <body>
            <h1>Umbra Control Deck</h1>
            <div class='container'>
        )html";

        for (auto cam : cameras) {
            html += "<div class='camera-row'>";
            
            html += "<div class='left-col'><div class='card'><h3>Camera stream: " + cam->name + "</h3><img src='/stream/" + cam->name + "' class='stream-img'></div></div>";
            
            html += "<div class='right-col'><div class='card'><div><h3>Tuning Panel: " + cam->name + "</h3>";
            
            std::string autoChecked = cam->auto_exposure ? "checked" : "";
            html += "<div class='row'><label>Auto Exposure</label>";
            html += "<input type='checkbox' id='" + cam->name + "_auto' " + autoChecked + " onchange=\"update('" + cam->name + "', 'auto', this.checked)\"></div>";

            html += "<div class='row'><label>Exposure</label><div class='slider-wrap'>";
            html += "<button class='step-btn' onclick=\"changeValue('" + cam->name + "', 'exp', -5)\">-</button>";
            html += "<input type='range' id='" + cam->name + "_exp_slider' min='1' max='300' value='" + std::to_string(cam->exposure) + "' oninput=\"update('" + cam->name + "', 'exp', this.value)\">";
            html += "<input type='number' id='" + cam->name + "_exp' value='" + std::to_string(cam->exposure) + "' onchange=\"update('" + cam->name + "', 'exp', this.value)\">";
            html += "<button class='step-btn' onclick=\"changeValue('" + cam->name + "', 'exp', 5)\">+</button></div></div>";

            html += "<div class='row'><label>Gain</label><div class='slider-wrap'>";
            html += "<button class='step-btn' onclick=\"changeValue('" + cam->name + "', 'gain', -5)\">-</button>";
            html += "<input type='range' id='" + cam->name + "_gain_slider' min='0' max='100' value='" + std::to_string(cam->gain) + "' oninput=\"update('" + cam->name + "', 'gain', this.value)\">";
            html += "<input type='number' id='" + cam->name + "_gain' value='" + std::to_string(cam->gain) + "' onchange=\"update('" + cam->name + "', 'gain', this.value)\">";
            html += "<button class='step-btn' onclick=\"changeValue('" + cam->name + "', 'gain', 5)\">+</button></div></div>";

            html += "<div class='row'><label>Brightness</label><div class='slider-wrap'>";
            html += "<button class='step-btn' onclick=\"changeValue('" + cam->name + "', 'bright', -4)\">-</button>";
            html += "<input type='range' id='" + cam->name + "_bright_slider' min='-64' max='64' value='" + std::to_string(cam->brightness) + "' oninput=\"update('" + cam->name + "', 'bright', this.value)\">";
            html += "<input type='number' id='" + cam->name + "_bright' value='" + std::to_string(cam->brightness) + "' onchange=\"update('" + cam->name + "', 'bright', this.value)\">";
            html += "<button class='step-btn' onclick=\"changeValue('" + cam->name + "', 'bright', 4)\">+</button></div></div>";

            html += "</div>";
            html += "<button onclick=\"save('" + cam->name + "')\">Save " + cam->name + " to Pi</button></div></div>";
            html += "</div>";
        }

        html += "</div></body></html>";
        res.set_content(html, "text/html");
    });

    svr.Get("/stream/(.*)", [&](const httplib::Request& req, httplib::Response& res) {
        std::string camName = req.matches[1]; clientIsWatching = true;
        res.set_chunked_content_provider("multipart/x-mixed-replace;boundary=frame", [camName](size_t offset, httplib::DataSink &sink) {
            if (!clientIsWatching) return false;
            cv::Mat frameToEncode;
            { std::lock_guard<std::mutex> lock(frameMutex); if (cameraFrames[camName].empty()) return true; frameToEncode = cameraFrames[camName].clone(); }
            std::vector<uchar> buf; cv::imencode(".jpg", frameToEncode, buf, {cv::IMWRITE_JPEG_QUALITY, 40});
            std::string head = "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + std::to_string(buf.size()) + "\r\n\r\n";
            sink.write(head.data(), head.size()); sink.write((const char*)buf.data(), buf.size()); sink.write("\r\n", 2);
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); return true;
        },[](bool success) { clientIsWatching = false; });
    });

    svr.Get("/set", [&](const httplib::Request& req, httplib::Response& res) {
        std::string camName = req.has_param("cam") ? req.get_param_value("cam") : "";
        for (auto cam : cameras) {
            if (camName.empty() || cam->name == camName) {
                if (req.has_param("auto")) {
                    bool autoMode = (req.get_param_value("auto") == "true");
                    cam->auto_exposure = autoMode;
                    if (autoMode) {
                        system(("v4l2-ctl -d " + cam->path + " -c auto_exposure=3").c_str());
                    } else {
                        system(("v4l2-ctl -d " + cam->path + " -c auto_exposure=1").c_str());
                        system(("v4l2-ctl -d " + cam->path + " -c exposure_time_absolute=" + std::to_string(cam->exposure)).c_str());
                    }
                }
                if (req.has_param("exp") && !cam->auto_exposure) {
                    int e = std::stoi(req.get_param_value("exp"));
                    system(("v4l2-ctl -d " + cam->path + " -c auto_exposure=1 -c exposure_time_absolute=" + std::to_string(e)).c_str());
                    cam->exposure = e;
                }
                if (req.has_param("gain")) {
                    int g = std::stoi(req.get_param_value("gain"));
                    system(("v4l2-ctl -d " + cam->path + " -c gain=" + std::to_string(g)).c_str());
                    cam->gain = g;
                }
                if (req.has_param("bright")) {
                    int b = std::stoi(req.get_param_value("bright"));
                    system(("v4l2-ctl -d " + cam->path + " -c brightness=" + std::to_string(b)).c_str());
                    cam->brightness = b;
                }
            }
        }
        res.set_content("ok", "text/plain");
    });

    svr.Get("/save", [&](const httplib::Request& req, httplib::Response& res) {
        std::string camName = req.has_param("cam") ? req.get_param_value("cam") : "";
        std::ifstream f(configPath); if (!f.is_open()) { res.set_content("err", "text/plain"); return; }
        json cfg = json::parse(f); f.close();
        for (auto& c_json : cfg["cameras"]) {
            std::string name = c_json["name"];
            if (camName.empty() || name == camName) {
                for (auto cam : cameras) {
                    if (cam->name == name) {
                        c_json["tuning"]["exposure"] = cam->exposure;
                        c_json["tuning"]["gain"] = cam->gain;
                        c_json["tuning"]["brightness"] = cam->brightness;
                        c_json["tuning"]["auto_exposure"] = cam->auto_exposure;
                    }
                }
            }
        }
        std::ofstream out(configPath); out << cfg.dump(4); out.close();
        res.set_content("saved", "text/plain");
    });

    svr.listen("0.0.0.0", 5801);
}

// --- CAMERA WORKER ---
void cameraWorker(CameraInstance* cam) {
    if (cam->auto_exposure) {
        system(("v4l2-ctl -d " + cam->path + " -c auto_exposure=3").c_str());
    } else {
        system(("v4l2-ctl -d " + cam->path + " -c auto_exposure=1 -c exposure_time_absolute=" + std::to_string(cam->exposure)).c_str());
    }
    system(("v4l2-ctl -d " + cam->path + " -c gain=" + std::to_string(cam->gain)).c_str());
    system(("v4l2-ctl -d " + cam->path + " -c brightness=" + std::to_string(cam->brightness)).c_str());

    apriltag_family_t *tf = tag36h11_create(); apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf); td->nthreads = 4; 
    td->quad_decimate = 2.0; 
    
    cv::TermCriteria subpixCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.01);
    cv::Mat frame, gray, subpix_gray, debugFrame;
    double s = TAG_SIZE / 2.0;

    // THE FIX: The exact working Tag Corners array that prevented the blue/red side switch
    std::vector<cv::Point3d> tag_corners_nwu = {
        {0, -s, -s}, // 0: BL 
        {0,  s, -s}, // 1: BR 
        {0,  s,  s}, // 2: TR 
        {0, -s,  s}  // 3: TL 
    };

    while (true) {
        std::string pipe = "v4l2src device=" + cam->path + " ! image/jpeg,width=1280,height=800,framerate=100/1 ! jpegdec ! videoconvert ! appsink max-buffers=1 drop=true";
        cv::VideoCapture cap(pipe, cv::CAP_GSTREAMER);
        if(!cap.isOpened()) { std::this_thread::sleep_for(std::chrono::seconds(2)); continue; }
        
        while (true) {
            if (!cap.read(frame) || frame.empty()) break;
            int64_t local_time_us = nt::Now();
            auto time_offset = nt::NetworkTableInstance::GetDefault().GetServerTimeOffset();
            double ts = (local_time_us + time_offset.value_or(0)) / 1000000.0;
            
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
            image_u8_t img = { .width = gray.cols, .height = gray.rows, .stride = gray.cols, .buf = gray.data };
            zarray_t *detections = apriltag_detector_detect(td, &img);
            
            if (clientIsWatching) {
                frame.copyTo(debugFrame);
                for (int i = 0; i < zarray_size(detections); i++) {
                    apriltag_detection_t *det; zarray_get(detections, i, &det);
                    for (int j=0; j<4; j++) cv::line(debugFrame, cv::Point(det->p[j][0], det->p[j][1]), cv::Point(det->p[(j+1)%4][0], det->p[(j+1)%4][1]), cv::Scalar(0,255,0), 2);
                }
                std::lock_guard<std::mutex> lock(frameMutex); cameraFrames[cam->name] = debugFrame.clone();
            }

            // ANTI-JITTER: Light Gaussian Blur to smooth out sensor noise for subpix
            cv::GaussianBlur(gray, subpix_gray, cv::Size(3, 3), 0.75);

            std::vector<cv::Point3d> objectPoints;
            std::vector<cv::Point2f> imagePoints;
            int tagsSeen = 0;
            int lastAcceptedTagId = -1;

            for (int i = 0; i < zarray_size(detections); i++) {
                apriltag_detection_t *det; zarray_get(detections, i, &det);
                if (fieldMap.count(det->id) && det->decision_margin > 50) {
                    std::vector<cv::Point2f> imgPts = {{(float)det->p[0][0],(float)det->p[0][1]},{(float)det->p[1][0],(float)det->p[1][1]},{(float)det->p[2][0],(float)det->p[2][1]},{(float)det->p[3][0],(float)det->p[3][1]}};
                    
                    if (cv::norm(imgPts[0] - imgPts[1]) > 25.0) {
                        try { cv::cornerSubPix(subpix_gray, imgPts, cv::Size(3, 3), cv::Size(-1, -1), subpixCriteria); } catch(...) {}
                    }
                    
                    std::vector<cv::Point2f> undistortedPts;
                    cv::undistortPoints(imgPts, undistortedPts, cam->camMatrix, cam->distCoeffs, cv::noArray(), cam->camMatrix);

                    cv::Mat T_field_tag = fieldMap[det->id].T_field_tag;
                    for (const auto& localPt : tag_corners_nwu) {
                        cv::Mat ptMat = (cv::Mat_<double>(4,1) << localPt.x, localPt.y, localPt.z, 1.0);
                        cv::Mat fieldPtMat = T_field_tag * ptMat;
                        objectPoints.push_back(cv::Point3d(fieldPtMat.at<double>(0,0), fieldPtMat.at<double>(1,0), fieldPtMat.at<double>(2,0)));
                    }
                    for (const auto& uPt : undistortedPts) imagePoints.push_back(uPt);
                    
                    lastAcceptedTagId = det->id;
                    tagsSeen++;
                }
            }

            if (tagsSeen > 1) { 
                cv::Mat rvec, tvec;
                if (cv::solvePnP(objectPoints, imagePoints, cam->camMatrix, cam->distCoeffs, rvec, tvec, false, cv::SOLVEPNP_SQPNP)) {
                    
                    // ANTI-JITTER: Levenberg-Marquardt refinement
                    cv::solvePnPRefineLM(objectPoints, imagePoints, cam->camMatrix, cam->distCoeffs, rvec, tvec);

                    cv::Mat R_f_c; cv::Rodrigues(rvec, R_f_c);
                    cv::Mat T_c_f_ocv = makeT(R_f_c, tvec).inv();

                    // THE FIX: The exact working swizzle that kept you on the correct side
                    cv::Mat M_nwu_to_ocv = (cv::Mat_<double>(4,4) << 0,-1,0,0,  0,0,-1,0,  1,0,0,0,  0,0,0,1);
                    cv::Mat M_ocv_to_nwu = M_nwu_to_ocv.inv();

                    cv::Mat T_cam_to_field_nwu = T_c_f_ocv * M_ocv_to_nwu;
                    cv::Mat T_f_r = T_cam_to_field_nwu * cam->T_robot_camera.inv();

                    double x = T_f_r.at<double>(0, 3);
                    double y = T_f_r.at<double>(1, 3);
                    double yaw = atan2(T_f_r.at<double>(1,0), T_f_r.at<double>(0,0));

                    // --- THE ALIGNMENT FIX ---
                    // Natively aligns your correct -90 yaw directly with the physical chassis arrow (+90 shift)
                    yaw = yaw + (M_PI / 2.0);

                    cam->pub.Set(std::vector<double>{ts, x, y, 0, 0, 0, yaw, (double)tagsSeen, cv::norm(tvec)});
                }
            } else {
                cam->pub.Set(std::vector<double>{ts, 0, 0, 0, 0, 0, 0, 0, 0});
            }
            apriltag_detections_destroy(detections);
        }
        cap.release();
    }
}

int main() {
    loadField("/home/dietpi/Umbra/field.json");
    std::ifstream f(configPath); if (!f.is_open()) return 1;
    json cfg = json::parse(f);
    auto inst = nt::NetworkTableInstance::GetDefault(); 
    inst.StartClient4("UmbraPi"); inst.SetServerTeam(cfg["team_number"]);
    
    std::vector<CameraInstance*> cameras;
    for (auto& c_json : cfg["cameras"]) {
        CameraInstance* cam = new CameraInstance(); cam->name = c_json["name"]; cam->path = c_json["path"];
        auto cal = c_json["calibration"]; cam->camMatrix = (cv::Mat_<double>(3,3) << cal["fx"].get<double>(), 0, cal["cx"].get<double>(), 0, cal["fy"].get<double>(), cal["cy"].get<double>(), 0, 0, 1);
        std::vector<double> d = cal["dist"].get<std::vector<double>>(); cam->distCoeffs = cv::Mat(d.size(), 1, CV_64F);
        for(int i=0; i < (int)d.size(); i++) cam->distCoeffs.at<double>(i) = d[i];
        
        auto off = c_json["offset"]; 
        cam->T_robot_camera = makeT(eulerToRotation(off["roll"].get<double>(), off["pitch"].get<double>(), off["yaw"].get<double>()), 
                                  (cv::Mat_<double>(3,1) << off["x"].get<double>(), off["y"].get<double>(), off["z"].get<double>()));
        
        // --- SECURE BACKWARD-COMPATIBLE INITIALIZATION ---
        cam->exposure = 25;
        cam->gain = 0;
        cam->brightness = 0;
        cam->auto_exposure = false;

        if (c_json.contains("tuning")) {
            auto t = c_json["tuning"];
            cam->exposure = t.value("exposure", 25);
            cam->gain = t.value("gain", 0);
            cam->brightness = t.value("brightness", 0);
            cam->auto_exposure = t.value("auto_exposure", false);
        }

        cam->pub = inst.GetTable("Umbra")->GetSubTable(cam->name)->GetDoubleArrayTopic("observations").Publish();
        cameras.push_back(cam);
    }
    
    std::thread uiThread(runWebServer, std::ref(cameras)); uiThread.detach();
    std::vector<std::thread> workers;
    for (auto c : cameras) workers.emplace_back(cameraWorker, c);
    for (auto& w : workers) w.join();
    return 0;
}
