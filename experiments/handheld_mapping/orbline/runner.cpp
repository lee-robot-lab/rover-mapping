// ================================================================
// runner.cpp
// 설명: rectified stereo 파일 또는 point stdin을 추적하고 pose·최종 keyframe을 내보낸다.
// 사용법: rover_orb3_compare ORBvoc LSDvoc settings.yaml pairs.tsv|- output_dir
// ================================================================
#include <System.h>
#include <KeyFrame.h>
#include <Map.h>
#include <MapPoint.h>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <vector>

#ifdef ROVER_POINT_BASELINE
// 오래된 runtime이 복구 설정을 조용히 무시하지 않도록 실행 시 지원을 확인한다.
extern "C" bool orbslam3_has_stereo_recovery_wait() __attribute__((weak));
extern "C" bool orbslam3_has_initial_stereo_relocalization() __attribute__((weak));
extern "C" bool orbslam3_has_safe_presave_observations() __attribute__((weak));

// 개별 표본의 시각은 Python 경계에서 카메라 clock으로 검증한다.
// 수신 telemetry 평균값이나 수신 시각으로 대체하지 않는다.
class ImuInput {
    std::ifstream input;
    bool available = false;
    long long stamp = -1, previous = -1;
    float values[6];
    void advance() {
        std::string line;
        available = static_cast<bool>(std::getline(input,line));
        if (!available) return;
        if (!line.empty() && line.back() == '\r') line.pop_back();
        std::istringstream row(line);
        std::string field;
        std::getline(row,field,',');
        size_t used = 0;
        stamp = std::stoll(field,&used);
        if (used != field.size() || stamp < 0 || stamp <= previous)
            throw std::runtime_error("invalid/nonmonotonic IMU timestamp");
        previous = stamp;
        for (int i=0; i<6; ++i) {
            if (!std::getline(row,field,',')) throw std::runtime_error("missing IMU field");
            values[i] = std::stof(field,&used);
            if (used != field.size() || !std::isfinite(values[i]))
                throw std::runtime_error("invalid IMU measurement");
        }
        if (std::getline(row,field,',')) throw std::runtime_error("extra IMU field");
    }
public:
    explicit ImuInput(const char* path) : input(path) {
        std::string header;
        std::getline(input,header);
        if (!header.empty() && header.back() == '\r') header.pop_back();
        if (header != "stamp_ns,ax,ay,az,gx,gy,gz")
            throw std::runtime_error("invalid IMU CSV header");
        advance();
        if (!available) throw std::runtime_error("empty IMU input");
    }
    std::vector<ORB_SLAM3::IMU::Point> take(long long camera_stamp) {
        std::vector<ORB_SLAM3::IMU::Point> points;
        while (available && stamp <= camera_stamp) {
            if (points.size() >= 16384) throw std::runtime_error("too many IMU samples per stereo frame");
            points.emplace_back(values[0],values[1],values[2],values[3],values[4],values[5],stamp*1e-9);
            advance();
        }
        if (points.empty()) throw std::runtime_error("no IMU samples for stereo frame");
        return points;
    }
};
#endif

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--capabilities") {
#ifdef ROVER_POINT_BASELINE
        std::cout << "{\"stereo\":true,\"offline_imu_stereo\":true,\"live_imu_stereo\":false}\n";
#else
        std::cout << "{\"offline_imu_stereo\":false}\n";
#endif
        return 0;
    }
    if (argc != 6 && argc != 8) { std::cerr << "ORBvoc LSDvoc yaml pairs.tsv output_dir [--imu samples.csv]\n"; return 2; }
    try {
        const bool live = std::string(argv[4]) == "-";
        const bool inertial = argc == 8;
        if (inertial && (std::string(argv[6]) != "--imu" || live))
            throw std::runtime_error("--imu requires an offline point-baseline recording");
#ifndef ROVER_POINT_BASELINE
        if (live || inertial) throw std::runtime_error("stdin live and IMU modes require point baseline");
#endif
#ifdef ROVER_POINT_BASELINE
        std::unique_ptr<ImuInput> imu;
        if (inertial) imu.reset(new ImuInput(argv[7]));
        {
            const cv::FileStorage settings(argv[3], cv::FileStorage::READ);
            if (!settings["Tracking.StereoRelocalizationWaitSeconds"].empty()
                && (!orbslam3_has_stereo_recovery_wait || !orbslam3_has_stereo_recovery_wait()
                    || !orbslam3_has_initial_stereo_relocalization || !orbslam3_has_initial_stereo_relocalization()))
                throw std::runtime_error("Stereo recovery patch missing or outdated: apply orbslam3_stereo_recovery_wait.patch and rebuild native runtime");
        }
        if (!orbslam3_has_safe_presave_observations || !orbslam3_has_safe_presave_observations())
            throw std::runtime_error("Atlas save safety patch missing: apply orbslam3_presave_observation_safety.patch and rebuild native runtime");
        ORB_SLAM3::System slam(argv[1], argv[3],
                              inertial ? ORB_SLAM3::System::IMU_STEREO : ORB_SLAM3::System::STEREO, false);
#else
        ORB_SLAM3::System slam(argv[1], argv[2], argv[3], ORB_SLAM3::System::STEREO, false);
#endif
        std::ifstream pair_file;
        std::istream* input = &std::cin;
        if (!live) { pair_file.open(argv[4]); input = &pair_file; }
        std::ofstream timing(std::string(argv[5]) + "/timings.csv");
        if (!*input || !timing) throw std::runtime_error("cannot open input/output");
        std::ofstream poses;
        if (live) {
            poses.open(std::string(argv[5]) + "/live_poses.csv");
            if (!poses) throw std::runtime_error("cannot open live poses");
            poses << "stamp_ns,state,map_id,process_ms";
            for (int i=0; i<4; ++i) for (int j=0; j<4; ++j) poses << ",m" << i << j;
            poses << '\n' << std::setprecision(17); poses.flush();
        }
        timing << "stamp_ns,state,map_id,process_ms\n" << std::setprecision(17);
        std::map<unsigned long, ORB_SLAM3::Map*> maps;
        std::string line;
        size_t processed = 0;
        while (std::getline(*input, line)) {
            std::istringstream row(line);
            std::string stamp, left, right;
            std::getline(row, stamp, '\t'); std::getline(row, left, '\t'); std::getline(row, right);
            cv::Mat image_left = cv::imread(left, cv::IMREAD_GRAYSCALE);
            cv::Mat image_right = cv::imread(right, cv::IMREAD_GRAYSCALE);
            if (image_left.empty() || image_right.empty()) throw std::runtime_error("missing stereo image");
            auto start = std::chrono::steady_clock::now();
#ifdef ROVER_POINT_BASELINE
            const auto measurements = inertial ? imu->take(std::stoll(stamp)) : std::vector<ORB_SLAM3::IMU::Point>();
            auto tracked_pose = slam.TrackStereo(image_left, image_right, std::stod(stamp)*1e-9,measurements);
#else
            slam.TrackStereo(image_left, image_right, std::stod(stamp)*1e-9);
#endif
            double milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-start).count();
            int state = slam.GetTrackingState();
            long long map_id = -1;
            if (state == 2) {
                for (auto* point : slam.GetTrackedMapPoints()) {
                    if (!point || point->isBad()) continue;
                    auto* map = point->GetMap();
                    if (!map || map->IsBad()) continue;
                    map_id = map->GetInitKFid(); maps[map_id] = map; break;
                }
            }
            timing << stamp << ',' << state << ',' << map_id << ',' << milliseconds << '\n';
            timing.flush();
#ifdef ROVER_POINT_BASELINE
            if (live) {
                const Eigen::Matrix4f world_camera = tracked_pose.inverse().matrix();
                poses << stamp << ',' << state << ',' << map_id << ',' << milliseconds;
                for (int i=0; i<4; ++i) for (int j=0; j<4; ++j) poses << ',' << world_camera(i,j);
                poses << '\n'; poses.flush();
                if (!poses) throw std::runtime_error("live pose write failed");
            }
#endif
            ++processed;
        }
        slam.Shutdown();
        size_t exported = 0;
        size_t imu_initialized_maps = 0, skipped_uninitialized_maps = 0;
        for (const auto& item : maps) {
            auto* map = item.second;
            if (map->IsBad()) continue;
#ifdef ROVER_POINT_BASELINE
            if (inertial) {
                if (!map->isImuInitialized()) { ++skipped_uninitialized_maps; continue; }
                ++imu_initialized_maps;
            }
#endif
            auto id = map->GetInitKFid();
            std::ofstream frames(std::string(argv[5]) + "/map_" + std::to_string(id) + "_keyframes.csv");
            frames << "map_id,keyframe_id,source_stamp,x,y,z,qx,qy,qz,qw\n" << std::setprecision(17);
            for (auto* frame : map->GetAllKeyFrames()) {
                if (!frame || frame->isBad() || frame->GetMap() != map) continue;
                Eigen::Matrix3d rotation;
#ifdef ROVER_POINT_BASELINE
                auto se3 = frame->GetPoseInverse();
                cv::Mat pose(4, 4, CV_32F);
                for (int i=0; i<4; ++i) for (int j=0; j<4; ++j) pose.at<float>(i,j) = se3.matrix()(i,j);
#else
                cv::Mat pose = frame->GetPoseInverse();
#endif
                for (int i=0; i<3; ++i) for (int j=0; j<3; ++j) rotation(i,j) = pose.at<float>(i,j);
                Eigen::Quaterniond q(rotation); q.normalize();
                frames << id << ',' << frame->mnId << ',' << frame->mTimeStamp;
                for (int i=0; i<3; ++i) frames << ',' << pose.at<float>(i,3);
                frames << ',' << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << '\n';
                ++exported;
            }
            if (!frames) throw std::runtime_error("keyframe export failed");
        }
        if (!timing || processed == 0 || exported == 0) throw std::runtime_error("no valid final keyframes");
        std::ofstream complete(std::string(argv[5]) + "/runner_complete.json");
        complete << "{\"status\":\"complete\",\"frames\":" << processed
                 << ",\"keyframes\":" << exported
                 << ",\"imu_initialized_maps\":" << imu_initialized_maps
                 << ",\"skipped_uninitialized_maps\":" << skipped_uninitialized_maps << "}\n";
        return complete ? 0 : 1;
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
