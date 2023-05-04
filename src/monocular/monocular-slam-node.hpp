#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

//#include "rclcpp/rclcpp.hpp"
//#include "sensor_msgs/msg/image.hpp"
//
//#include <cv_bridge/cv_bridge.h>
//
//#include "System.h"
//#include "Frame.h"
//#include "Map.h"
//#include "Tracking.h"

#include "utility.hpp"
#include "Converter.h"

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <cv_bridge/cv_bridge.h>

#include"System.h"
#include"Frame.h"
#include "Map.h"
#include "Tracking.h"

class MonocularSlamNode : public rclcpp::Node
{
public:
        MonocularSlamNode(ORB_SLAM3::System* pSLAM, const string &strVocFile, const string &strSettingsFile);
    ~MonocularSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;
    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);
    ORB_SLAM3::System* m_SLAM;
    void UpdateSLAMState();
    void UpdateMapState();
    void PublishFrame();
    void PublishCurrentCamera();
    void InitializeMarkersPublisher( const string &strSettingPath);
    void PublishMapPoints();
    void PublishKeyFrames();
    cv::Mat DrawFrame();
    sensor_msgs::msg::PointCloud2 MapPointsToPointCloud (std::vector<ORB_SLAM3::MapPoint*> map_points);

    void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);
    std::mutex mMutex;
    cv_bridge::CvImagePtr m_cvImPtr;
    cv::Mat Tcw;
    int N;
    vector<cv::KeyPoint> mvCurrentKeys;
    vector<bool> mvbMap, mvbVO;
    int mnTracked, mnTrackedVO;
    vector<cv::KeyPoint> mvIniKeys;
    vector<int> mvIniMatches;
    vector<ORB_SLAM3::KeyFrame*> mvKeyFrames;
    vector<ORB_SLAM3::MapPoint*> mvMapPoints;
    vector<ORB_SLAM3::MapPoint*> mvRefMapPoints;
    visualization_msgs::msg::Marker mPoints;
    visualization_msgs::msg::Marker mReferencePoints;
    visualization_msgs::msg::Marker mKeyFrames;
    visualization_msgs::msg::Marker mReferenceKeyFrames;
    visualization_msgs::msg::Marker mCovisibilityGraph;
    visualization_msgs::msg::Marker mMST;
    visualization_msgs::msg::Marker mCurrentCamera;

    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

    int mState;
    bool mbOnlyTracking;
    bool mbUpdated;
    bool mbCameraUpdated;
    bool mbMapUpdated;

    std::string map_frame_id_param_;
    std::string camera_frame_id_param_;
    std::string target_frame_id_param_;
    std::string map_file_name_param_;
    std::string voc_file_name_param_;
    bool load_map_param_;
    bool publish_pointcloud_param_;
    bool publish_tf_param_;
    bool publish_pose_param_;
    int min_observations_per_point_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_annotated_image_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_map_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_publisher_;

    sensor_msgs::msg::Image::SharedPtr msg_;
        rclcpp::Time current_frame_time_;

};

#endif
