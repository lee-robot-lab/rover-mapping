// ================================================================
// presave_probe.cpp
// 설명: 실제 native 지도에서 소속이 바뀐 관측과 마지막 관측 제거의 저장 안전성을 검사한다.
// 사용법: presave_probe ORBvoc settings.yaml left.png right.png foreign|last|missing-keyframe
// ================================================================
#include <System.h>
#include <KeyFrame.h>
#include <Map.h>
#include <MapPoint.h>
#include <opencv2/opencv.hpp>
#include <set>
#include <stdexcept>
#include <string>

int main(int argc, char** argv) {
    if(argc != 6) return 2;
    ORB_SLAM3::System slam(argv[1], argv[2], ORB_SLAM3::System::STEREO, false);
    slam.TrackStereo(cv::imread(argv[3], 0), cv::imread(argv[4], 0), 0.0);
    ORB_SLAM3::MapPoint* point = nullptr;
    for(auto* p : slam.GetTrackedMapPoints())
        if(p && !p->isBad()) { point = p; break; }
    if(!point) throw std::runtime_error("no initial map point");
    auto* map = point->GetMap();
    auto* frame = point->GetReferenceKeyFrame();
    slam.Shutdown();
    if(std::string(argv[5]) == "last") {
        point->EraseObservation(frame);
        if(!point->GetObservations().empty() || point->GetReferenceKeyFrame() != nullptr)
            throw std::runtime_error("last observation must clear the reference keyframe");
    } else if(std::string(argv[5]) == "missing-keyframe") {
        std::set<ORB_SLAM3::KeyFrame*> frames;
        std::set<ORB_SLAM3::MapPoint*> points{point};
        point->PreSave(frames, points);
        if(!point->GetObservations().empty() || point->GetReferenceKeyFrame() != nullptr)
            throw std::runtime_error("missing keyframe must clear its observation and reference");
    } else if(std::string(argv[5]) == "foreign") {
        std::set<ORB_SLAM3::GeometricCamera*> cameras;
        const auto before = map->MapPointsInMap();
        map->PreSave(cameras);
        if(before == 0 || map->MapPointsInMap() != before || point->GetReferenceKeyFrame() != frame)
            throw std::runtime_error("valid observations must survive save cleanup");
        ORB_SLAM3::Map foreign(999);
        frame->UpdateMap(&foreign);
        map->PreSave(cameras);
        if(map->MapPointsInMap() != 0)
            throw std::runtime_error("foreign observations must remove orphaned map points");
    } else return 2;
    return 0;
}
