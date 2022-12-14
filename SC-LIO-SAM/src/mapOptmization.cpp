#include "utility.h"

#include "lio_sam/cloud_info.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>

#include "Scancontext.h"
//**************************
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <pcl_ros/point_cloud.h>
#include <opencv2/core/core.hpp>

//平面约束需要的头文件
#include <gtsam/slam/OrientedPlane3Factor.h>
#include <gtsam/geometry/OrientedPlane3.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
//计算平面法向量部分代码而添加的
#include <pcl/common/centroid.h>
#include <Eigen/Eigenvalues>

#include "src/efficient_online_segmentation/efficient_online_segmentation.h"
//*************************


using namespace gtsam;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::G; // GPS pose
using symbol_shorthand::P; //Plane (x,y,z,d) OrientedPlane3

void saveOptimizedVerticesKITTIformat(gtsam::Values _estimates, std::string _filename)
{
    using namespace gtsam;

    // ref from gtsam's original code "dataset.cpp"
    std::fstream stream(_filename.c_str(), fstream::out);

    for(const auto& key_value: _estimates) {
        auto p = dynamic_cast<const GenericValue<Pose3>*>(&key_value.value);
        if (!p) continue;

        const Pose3& pose = p->value();

        Point3 t = pose.translation();
        Rot3 R = pose.rotation();
        auto col1 = R.column(1); // Point3
        auto col2 = R.column(2); // Point3
        auto col3 = R.column(3); // Point3

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " "
               << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y() << " "
               << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << std::endl;
    }
}


/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;                  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

typedef PointXYZIRPYT  PointTypePose;

// giseop
enum class SCInputType { 
    SINGLE_SCAN_FULL, 
    SINGLE_SCAN_FEAT, 
    MULTI_SCAN_FEAT 
}; 

class mapOptimization : public ParamServer
{

public:

    //plane**********
    SegmentationParams params;

    // gtsam
    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;
    Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance;

    ros::Publisher pubLaserCloudSurround;
    ros::Publisher pubLaserOdometryGlobal;
    ros::Publisher pubLaserOdometryIncremental;
    ros::Publisher pubKeyPoses;
    ros::Publisher pubPath;

    ros::Publisher pubHistoryKeyFrames;
    ros::Publisher pubIcpKeyFrames;
    ros::Publisher pubRecentKeyFrames;
    ros::Publisher pubRecentKeyFrame;
    ros::Publisher pubCloudRegisteredRaw;
    ros::Publisher pubLoopConstraintEdge;
    //for plane
    ros::Publisher pubRecentPlanePoints;

    ros::Subscriber subCloud;
    ros::Subscriber subGPS;
    ros::Subscriber subLoop;

    std::deque<nav_msgs::Odometry> gpsQueue;
    lio_sam::cloud_info cloudInfo;

    vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
    vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;

    //for plane
    std::deque<Eigen::Vector4d> planeCurrentQueue;
    std::deque<Eigen::Vector4d> planeSubmapQueue;

    vector<pcl::PointCloud<PointType>::Ptr> planeCloudKeyFrames;
    
    pcl::PointCloud<PointXYZIRT>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
    pcl::PointCloud<PointXYZIRT>::Ptr copy_cloudKeyPoses3D;
    pcl::PointCloud<PointXYZIRT>::Ptr copy_cloudKeyPoses2D; // giseop 
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;

    pcl::PointCloud<PointType>::Ptr laserCloudRaw; // giseop
    pcl::PointCloud<PointType>::Ptr laserCloudRawDS; // giseop

    float dectectPlaneCounts;
    int plane_Index = 0;
    bool flagForPlaneExtract;//该flag为true表示该关键帧进入了平面点提取  作标记用 用来得到局部地面点（世界系下）
    bool flagForSubPlane;  //该flag为true表示所提取的局部地图帧中有提取过地面点的帧，为false表示没有，则不进行计算局部地图法向量操作
    //plane*******************************************************************************

    // Ros tools.
    ros::Publisher segmted_cloud_publisher;
    ros::Publisher range_image_publisher;
    ros::Publisher extracted_lines_publisher;
    ros::Publisher transformed_cloud_publisher;

    EfficientOnlineSegmentation efficient_sgmtt_;
    pcl::PointCloud<PointType>::Ptr common_original_cloud_;//plane
    pcl::PointCloud<PointXYZIRT>::Ptr custom_original_cloud_;//plane

    std::vector<int> labels_;

    pcl::PointCloud<PointType>::Ptr ground_points;


    //******************************************************************************
    double laserCloudRawTime;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast; // corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast; // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS; // downsampled corner featuer set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS; // downsampled surf featuer set from odoOptimization

    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    std::vector<PointType> laserCloudOriCornerVec; // corner point holder for parallel computation
    std::vector<PointType> coeffSelCornerVec;
    std::vector<bool> laserCloudOriCornerFlag;
    std::vector<PointType> laserCloudOriSurfVec; // surf point holder for parallel computation
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;
    //这个容器的目的是：局部地图点在每个关键帧处理一开始都要清空，而当前关键帧需要的局部地图点，有很大一部分在上一时刻已经
    //计算过了（将雷达坐标系下的点转到世界坐标系下），也就是这些点在当前帧不用重复处理，直接通过容器可以加入到当前帧所属的
    //局部地图中。
    map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> laserCloudMapContainer;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

    //局部地图平面相关
    map<int, pcl::PointCloud<PointType>> laserCloudPlaneMapContainer;
    pcl::PointCloud<PointType>::Ptr laserCloudPlaneFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudPlaneFromMapDS;
    pcl::KdTreeFLANN<PointXYZIRT>::Ptr kdtreeSurroundingKeyPosesForPlane;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

    pcl::KdTreeFLANN<PointXYZIRT>::Ptr kdtreeSurroundingKeyPoses;
    pcl::KdTreeFLANN<PointXYZIRT>::Ptr kdtreeHistoryKeyPoses;

    pcl::VoxelGrid<PointType> downSizeFilterSC; // giseop
    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    pcl::VoxelGrid<PointXYZIRT> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization
    
    //for plane 
    pcl::VoxelGrid<PointType> downSizeFilterPlane;
    pcl::VoxelGrid<PointXYZIRT> downSizeFilterSurroundingKeyPosesForPlane;

    ros::Time timeLaserInfoStamp;
    double timeLaserInfoCur;
    double simTime;

    float transformTobeMapped[6];

    std::mutex mtx;
    std::mutex mtxLoopInfo;

    bool isDegenerate = false;
    Eigen::Matrix<float, 6, 6> matP;

    int laserCloudCornerFromMapDSNum = 0;
    int laserCloudSurfFromMapDSNum = 0;
    int laserCloudCornerLastDSNum = 0;
    int laserCloudSurfLastDSNum = 0;

    bool aLoopIsClosed = false;
    // map<int, int> loopIndexContainer; // from new to old
    multimap<int, int> loopIndexContainer; // from new to old // giseop 

    vector<pair<int, int>> loopIndexQueue;
    vector<gtsam::Pose3> loopPoseQueue;
    // vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue; // Diagonal <- Gausssian <- Base
    vector<gtsam::SharedNoiseModel> loopNoiseQueue; // giseop for polymorhpisam (Diagonal <- Gausssian <- Base)

    deque<std_msgs::Float64MultiArray> loopInfoVec;

    nav_msgs::Path globalPath;

    Eigen::Affine3f transPointAssociateToMap;
    Eigen::Affine3f incrementalOdometryAffineFront;
    Eigen::Affine3f incrementalOdometryAffineBack;

    // // loop detector 
    SCManager scManager;

    // data saver
    std::fstream pgSaveStream; // pg: pose-graph 
    std::fstream pgTimeSaveStream; // pg: pose-graph 
    std::vector<std::string> edges_str;
    std::vector<std::string> vertices_str;
    // std::fstream pgVertexSaveStream;
    // std::fstream pgEdgeSaveStream;

    std::string saveSCDDirectory;
    std::string saveNodePCDDirectory;
    std::string saveCornerPCDDirectory;
    std::string saveSurfPCDDirectory;
public:
    mapOptimization()
    {
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = new ISAM2(parameters);

        LoadAlgorithmParams(nh, params);
        bool is_params_legal = params.UpdateInternalParams();
        if (!is_params_legal) {
            std::cout << "###################### ERROR! ###################### " << std::endl;
            std::cout << "## Parameters illegal, please check!!! " << std::endl;
            std::cout << "###################### ERROR! ###################### " << std::endl 
                << std::endl << std::endl << std::endl;
            ros::shutdown();
        }
        //plane******************************************************************************
        segmted_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>
            (pub_cloud_topic, 1);//当前帧地面点和墙面点
        range_image_publisher = nh.advertise<sensor_msgs::Image>
            (pub_rangeimage_topic, 1);
        extracted_lines_publisher = nh.advertise<visualization_msgs::MarkerArray>
            (pub_extractedlines_topic,1);
        transformed_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>
            ("/EOS_transformed_cloud", 1);//在base_link坐标系下的点云，未分类

        efficient_sgmtt_.ResetParameters(params);

        common_original_cloud_.reset(new pcl::PointCloud<PointType>());
        custom_original_cloud_.reset(new pcl::PointCloud<PointXYZIRT>());
        ground_points.reset(new pcl::PointCloud<PointType>());
        std::cout << "************************* have reset cloud************************" << std::endl;
        //*****************************************************************************************

        pubKeyPoses                 = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/trajectory", 1);
        pubLaserCloudSurround       = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/map_global", 1);//全局地图 --世界坐标系下
        pubLaserOdometryGlobal      = nh.advertise<nav_msgs::Odometry> ("lio_sam/mapping/odometry", 1);//图优化后的全局位姿
        pubLaserOdometryIncremental = nh.advertise<nav_msgs::Odometry> ("lio_sam/mapping/odometry_incremental", 1);
        pubPath                     = nh.advertise<nav_msgs::Path>("lio_sam/mapping/path", 1);

        subCloud = nh.subscribe<lio_sam::cloud_info>("lio_sam/feature/cloud_info", 2, &mapOptimization::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
        subGPS   = nh.subscribe<nav_msgs::Odometry> (gpsTopic, 200, &mapOptimization::gpsHandler, this, ros::TransportHints().tcpNoDelay());
        subLoop  = nh.subscribe<std_msgs::Float64MultiArray>("lio_loop/loop_closure_detection", 1, &mapOptimization::loopInfoHandler, this, ros::TransportHints().tcpNoDelay());

        pubHistoryKeyFrames   = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/icp_loop_closure_history_cloud", 1);//参与回环检测的候选关键帧的前后各25帧组成的点云  --世界坐标系下
        pubIcpKeyFrames       = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/icp_loop_closure_corrected_cloud", 1);//参与回环检测的当前关键帧下的点云（世界坐标系下），通过icp得到的Tcp，将这些点云与参考帧的点云对齐
        pubLoopConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>("/lio_sam/mapping/loop_closure_constraints", 1);

        pubRecentKeyFrames    = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/map_local", 1);//当前帧局部地图点（平面点，经过降采样）--世界坐标系下
        pubRecentKeyFrame     = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/cloud_registered", 1);//当前帧的平面点和角点（经过降采样）  --世界坐标系下
        pubCloudRegisteredRaw = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/cloud_registered_raw", 1);//当前帧的所有点（未将采样） --世界坐标系下

        //for plane
        pubRecentPlanePoints  =  nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/plane_local", 1);//当前帧局部地图点（地面点，经过降采样） --世界坐标系下

        const float kSCFilterSize = 0.5; // giseop
        downSizeFilterSC.setLeafSize(kSCFilterSize, kSCFilterSize, kSCFilterSize); // giseop

        downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization
        //for plane
        downSizeFilterPlane.setLeafSize(groundSubmapLeafSize, groundSubmapLeafSize, groundSubmapLeafSize);
        downSizeFilterSurroundingKeyPosesForPlane.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity);
        allocateMemory();

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

        // giseop
        // create directory and remove old files;
        // savePCDDirectory = std::getenv("HOME") + savePCDDirectory; // rather use global path 
        int unused = system((std::string("exec rm -r ") + savePCDDirectory).c_str());
        unused = system((std::string("mkdir ") + savePCDDirectory).c_str());

        saveSCDDirectory = savePCDDirectory + "SCDs/"; // SCD: scan context descriptor 
        unused = system((std::string("exec rm -r ") + saveSCDDirectory).c_str());
        unused = system((std::string("mkdir -p ") + saveSCDDirectory).c_str());

        saveNodePCDDirectory = savePCDDirectory + "Scans/";
        unused = system((std::string("exec rm -r ") + saveNodePCDDirectory).c_str());
        unused = system((std::string("mkdir -p ") + saveNodePCDDirectory).c_str());

        saveCornerPCDDirectory = savePCDDirectory + "Corner/";
        unused = system((std::string("exec rm -r ") + saveCornerPCDDirectory).c_str());
        unused = system((std::string("mkdir -p ") + saveCornerPCDDirectory).c_str());

        saveSurfPCDDirectory = savePCDDirectory + "Surf/";
        unused = system((std::string("exec rm -r ") + saveSurfPCDDirectory).c_str());
        unused = system((std::string("mkdir -p ") + saveSurfPCDDirectory).c_str());

        pgSaveStream = std::fstream(savePCDDirectory + "singlesession_posegraph.g2o", std::fstream::out);
        pgTimeSaveStream = std::fstream(savePCDDirectory + "times.txt", std::fstream::out); pgTimeSaveStream.precision(dbl::max_digits10);
        // pgVertexSaveStream = std::fstream(savePCDDirectory + "singlesession_vertex.g2o", std::fstream::out);
        // pgEdgeSaveStream = std::fstream(savePCDDirectory + "singlesession_edge.g2o", std::fstream::out);

    }

    //plane
    bool LoadAlgorithmParams(::ros::NodeHandle& node_handle, SegmentationParams& params)
    {
        // load params need no conversion.
        node_handle.param<int>("kLidarRows", params.kLidarRows, params.kLidarRows);
        node_handle.param<int>("kLidarCols", params.kLidarCols, params.kLidarCols);
        node_handle.param<int>("kNumSectors", params.kNumSectors, params.kNumSectors);
        node_handle.param<float>("kGroundYInterceptTolerance", params.kGroundYInterceptTolerance, 
                                                        params.kGroundYInterceptTolerance);
        node_handle.param<float>("kGroundPointLineDistThres", params.kGroundPointLineDistThres, 
                                                        params.kGroundPointLineDistThres);
        node_handle.param<int>("kWallLineMinBinNum", params.kWallLineMinBinNum, 
                                                    params.kWallLineMinBinNum);
        node_handle.param<float>("kWallPointLineDistThres", params.kWallPointLineDistThres, 
                                                        params.kWallPointLineDistThres);
        std::vector<float> ext_trans_vec, ext_rot_vec;
        node_handle.param<std::vector<float>>("kExtrinsicTrans", ext_trans_vec, std::vector<float>());
        node_handle.param<std::vector<float>>("kExtrinsicRot", ext_rot_vec, std::vector<float>());
        params.kExtrinsicTrans = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(ext_trans_vec.data(), 3, 1);
        params.kExtrinsicRot = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(ext_rot_vec.data(), 3, 3);

        // load params in degree (need to be converted).
        LoadAndConvertDegToRad(node_handle, "kLidarHorizRes", params.kLidarHorizRes);
        LoadAndConvertDegToRad(node_handle, "kLidarVertRes", params.kLidarVertRes);
        LoadAndConvertDegToRad(node_handle, "kLidarVertFovMax", params.kLidarVertFovMax);
        LoadAndConvertDegToRad(node_handle, "kLidarVertFovMin", params.kLidarVertFovMin);
        LoadAndConvertDegToRad(node_handle, "kLidarProjectionError", params.kLidarProjectionError);
        LoadAndConvertDegToSlope(node_handle, "kGroundSameLineTolerance", params.kGroundSameLineTolerance);
        LoadAndConvertDegToSlope(node_handle, "kGroundSlopeTolerance", params.kGroundSlopeTolerance);
        LoadAndConvertDegToSlope(node_handle, "kWallSameLineTolerance", params.kWallSameLineTolerance);
        LoadAndConvertDegToSlope(node_handle, "kWallSlopeTolerance", params.kWallSlopeTolerance);

        return true;
    }

    void allocateMemory()
    {
        cloudKeyPoses3D.reset(new pcl::PointCloud<PointXYZIRT>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointXYZIRT>());
        copy_cloudKeyPoses2D.reset(new pcl::PointCloud<PointXYZIRT>());
        copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        //for plane
        kdtreeSurroundingKeyPosesForPlane.reset(new pcl::KdTreeFLANN<PointXYZIRT>());

        kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointXYZIRT>());
        kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointXYZIRT>());

        laserCloudRaw.reset(new pcl::PointCloud<PointType>()); // giseop
        laserCloudRawDS.reset(new pcl::PointCloud<PointType>()); // giseop

        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>()); // corner feature set from odoOptimization
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>()); // surf feature set from odoOptimization
        laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled corner featuer set from odoOptimization
        laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled surf featuer set from odoOptimization

        laserCloudOri.reset(new pcl::PointCloud<PointType>());
        coeffSel.reset(new pcl::PointCloud<PointType>());

        laserCloudOriCornerVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelCornerVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriCornerFlag.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelSurfVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfFlag.resize(N_SCAN * Horizon_SCAN);

        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);

        laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

        //for Plane
        laserCloudPlaneFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudPlaneFromMapDS.reset(new pcl::PointCloud<PointType>());

        kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

        for (int i = 0; i < 6; ++i){
            transformTobeMapped[i] = 0;
        }

        dectectPlaneCounts = 0.0;
        flagForPlaneExtract = false;
        flagForSubPlane = false;

        matP.setZero();
    }

    void writeVertex(const int _node_idx, const gtsam::Pose3& _initPose)
    {
        gtsam::Point3 t = _initPose.translation();
        gtsam::Rot3 R = _initPose.rotation();

        std::string curVertexInfo {
            "VERTEX_SE3:QUAT " + std::to_string(_node_idx) + " "
            + std::to_string(t.x()) + " " + std::to_string(t.y()) + " " + std::to_string(t.z())  + " " 
            + std::to_string(R.toQuaternion().x()) + " " + std::to_string(R.toQuaternion().y()) + " " 
            + std::to_string(R.toQuaternion().z()) + " " + std::to_string(R.toQuaternion().w()) };

        // pgVertexSaveStream << curVertexInfo << std::endl;
        vertices_str.emplace_back(curVertexInfo);
    }
    
    void writeEdge(const std::pair<int, int> _node_idx_pair, const gtsam::Pose3& _relPose)
    {
        gtsam::Point3 t = _relPose.translation();
        gtsam::Rot3 R = _relPose.rotation();

        std::string curEdgeInfo {
            "EDGE_SE3:QUAT " + std::to_string(_node_idx_pair.first) + " " + std::to_string(_node_idx_pair.second) + " "
            + std::to_string(t.x()) + " " + std::to_string(t.y()) + " " + std::to_string(t.z())  + " " 
            + std::to_string(R.toQuaternion().x()) + " " + std::to_string(R.toQuaternion().y()) + " " 
            + std::to_string(R.toQuaternion().z()) + " " + std::to_string(R.toQuaternion().w()) };

        // pgEdgeSaveStream << curEdgeInfo << std::endl;
        edges_str.emplace_back(curEdgeInfo);
    }

    // void writeEdgeStr(const std::pair<int, int> _node_idx_pair, const gtsam::Pose3& _relPose, const gtsam::SharedNoiseModel _noise)
    // {
    //     gtsam::Point3 t = _relPose.translation();
    //     gtsam::Rot3 R = _relPose.rotation();

    //     std::string curEdgeSaveStream;
    //     curEdgeSaveStream << "EDGE_SE3:QUAT " << _node_idx_pair.first << " " << _node_idx_pair.second << " "
    //         << t.x() << " "  << t.y() << " " << t.z()  << " " 
    //         << R.toQuaternion().x() << " " << R.toQuaternion().y() << " " << R.toQuaternion().z()  << " " << R.toQuaternion().w() << std::endl;

    //     edges_str.emplace_back(curEdgeSaveStream);
    // }

    void laserCloudInfoHandler(const lio_sam::cloud_infoConstPtr& msgIn)
    {
        // extract time stamp
        timeLaserInfoStamp = msgIn->header.stamp;
        timeLaserInfoCur = msgIn->header.stamp.toSec();

        static double systemBegintime = timeLaserInfoCur;
        simTime = timeLaserInfoCur - systemBegintime;

        // extract info and feature cloud
        cloudInfo = *msgIn;
        pcl::fromROSMsg(msgIn->cloud_corner,  *laserCloudCornerLast);
        pcl::fromROSMsg(msgIn->cloud_surface, *laserCloudSurfLast);
        pcl::fromROSMsg(msgIn->cloud_deskewed,  *laserCloudRaw); // giseop
        laserCloudRawTime = cloudInfo.header.stamp.toSec(); // giseop save node time

        static double timeLastProcessing = -1;
        static double timeLastPlane = timeLaserInfoCur;//用于设置平面提取的频率
        std::ofstream lidarTime("/home/gky/桌面/slam_results/map_time.csv", std::ios::app);
        std::ofstream lidarTimedelta("/home/gky/桌面/slam_results/map_delta_time.csv", std::ios::app);
        lidarTime.precision(6);
        lidarTimedelta.precision(6);
        lidarTime << timeLaserInfoCur << endl; 
        lidarTimedelta << timeLaserInfoCur - timeLastProcessing << endl;

        std::lock_guard<std::mutex> lock(mtx);
        if (timeLaserInfoCur - timeLastProcessing >= mappingProcessInterval)
        {
            auto mapTimeStart = ros::Time::now().toSec();
            timeLastProcessing = timeLaserInfoCur;

            updateInitialGuess();

            extractSurroundingKeyFrames();

            downsampleCurrentScan();

            scan2MapOptimization();

            //平面点提取 这里设置频率为1Hz
            if(simTime > 7 && (timeLaserInfoCur - timeLastPlane) > groundExtractProcessInterval && saveFrame() == true)
            {
                dectectPlaneCounts += 1.0;
                std::cout << "simTime: " << simTime << std::endl;
                flagForPlaneExtract = true;
                timeLastPlane = timeLaserInfoCur;
                segmentPlane(msgIn);
                extractSubMapforPlane();

                 //****************************for plane****************************
                // save all the received plane points
                std::cout << "before add to submap, ground_points nums: " << ground_points->size() << std::endl;
                pcl::PointCloud<PointType>::Ptr thisPlaneKeyFrame(new pcl::PointCloud<PointType>());
                pcl::copyPointCloud(*ground_points,  *thisPlaneKeyFrame);
                //用当前帧地面点的第0个点的intensity来记录该地面帧在全局帧中的索引，用来之后转化点云的坐标系
                thisPlaneKeyFrame->points[0].intensity = cloudKeyPoses3D->size();
                // save key frame cloud
                planeCloudKeyFrames.push_back(thisPlaneKeyFrame);
                ground_points->clear();
                //ground_points.reset(new pcl::PointCloud<PointType>());//由于我添加地面点是通过push_back添加的
                                                                                                                                    //而角点和平面点时通过filter函数覆盖的，因此不用reset
    
                //计算平面法向量*************************** 平均花费时长约为1.7ms
                if(flagForSubPlane == true){

                    std::cout << "before filter, nums of plane points for submap: " << laserCloudPlaneFromMap->size() << std::endl;
                    downSizeFilterPlane.setInputCloud(laserCloudPlaneFromMap);
                    downSizeFilterPlane.filter(*laserCloudPlaneFromMapDS);
                    std::cout << "after filter, nums of plane points for submap: " << laserCloudPlaneFromMapDS->size() << std::endl;
                    clock_t time_plane_vector_start = clock();
	                Eigen::Vector4f zx;//中心 齐次坐标 第4维是1
	                pcl::compute3DCentroid(*laserCloudPlaneFromMapDS, zx); //xyz1
                    std::cout << "submap centor point: " << "(" << zx(0) << ", " << zx(1) << ", " << zx(2) << ", " << zx(3) <<  ")" << std::endl;
	                Eigen::Matrix3f xiefancha;
	                pcl::computeCovarianceMatrixNormalized(*laserCloudPlaneFromMapDS, zx, xiefancha);//计算归一化协方差矩阵
                    std::cout << "submap CovarianceMatrixNormalized: " << std::endl << xiefancha << std::endl;
	                //计算特征值和特征向量
	                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solve(xiefancha, Eigen::ComputeEigenvectors);
	                Eigen::Vector3f values = solve.eigenvalues();//特征值
	                Eigen::Matrix3f vectors = solve.eigenvectors();//特征向量 列向量为特征向量 其中第一列为最小特征值对应的特征向量
                    clock_t time_plane_vector_end = clock();
                    float time_plane_vector = (float(time_plane_vector_end - time_plane_vector_start))/CLOCKS_PER_SEC;
                    std::cout << "calculate submap plane vector take: " << time_plane_vector * 1000<< "ms" << std::endl;
	                //平面的法向量
	                double V_x = vectors(0, 0);
	                double V_y = vectors(1, 0);
	                double V_z = vectors(2, 0);
                    double x0 = zx(0);
                    double y0 = zx(1);
                    double z0 = zx(2);
                    double d = - (V_x * x0 + V_y * y0 + V_z * z0); 
                    std::cout << "submap plane normal vector: " << "(" << V_x << ", " << V_y << ", " << V_z << ", " << d << ")" << std::endl;
                    //*****************************************************************
                    if(V_z < 0){
                        Eigen::Vector4d currentPlane;
                        currentPlane << -V_x, -V_y, -V_z, -d;
                        planeSubmapQueue.push_back(currentPlane);
                    }else{
                        Eigen::Vector4d currentPlane;
                        currentPlane << V_x, V_y, V_z, d;
                        planeSubmapQueue.push_back(currentPlane);
                    }
                }
            }
            saveKeyFramesAndFactor();

            correctPoses();
            //std::cout << "correctPoses end" << std::endl;
            publishOdometry();
            //std::cout << "publishOdometry end" << std::endl;
            publishFrames();
            auto mapTimeEnd = ros::Time::now().toSec();
            ROS_INFO_STREAM("\033[1;36m" << setw(22) << "map takes" << (mapTimeEnd - mapTimeStart) *  1000 << "ms\033[0m");
            std::ofstream mapTaketime("/home/gky/桌面/slam_results/map_take_time.csv", std::ios::app);
            mapTaketime << (mapTimeEnd - mapTimeStart) *  1000 <<endl;
        }
    }

    void extractSubMapforPlane()
    {
        if (cloudKeyPoses3D->points.empty() == true)
            return;
        //必须有过地面点才进行提取局部地图地面点操作
        
        extractNearbyForPlane();
    }
    void extractNearbyForPlane()
    {
        pcl::PointCloud<PointXYZIRT>::Ptr surroundingKeyPosesForPlane(new pcl::PointCloud<PointXYZIRT>());
        pcl::PointCloud<PointXYZIRT>::Ptr surroundingKeyPosesForPlaneDS(new pcl::PointCloud<PointXYZIRT>());
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        // extract all the nearby key poses and downsample them
        kdtreeSurroundingKeyPosesForPlane->setInputCloud(cloudKeyPoses3D); // create kd-tree 搜索到的最近点包含自己
        kdtreeSurroundingKeyPosesForPlane->radiusSearch(cloudKeyPoses3D->back(), (double)surroundingKeyframeSearchRadiusForPlane, pointSearchInd, pointSearchSqDis);
        for (int i = 0; i < (int)pointSearchInd.size(); ++i)
        {
            //std::cout << "pointSearchInd.size(): "<< pointSearchInd.size() << std::endl;
            int id = pointSearchInd[i];
            surroundingKeyPosesForPlane->push_back(cloudKeyPoses3D->points[id]);
        }

        downSizeFilterSurroundingKeyPosesForPlane.setInputCloud(surroundingKeyPosesForPlane);
        downSizeFilterSurroundingKeyPosesForPlane.filter(*surroundingKeyPosesForPlaneDS);

        // also extract some latest key frames in case the robot rotates in one position
        int numPoses = cloudKeyPoses3D->size();
        for (int i = numPoses-1; i >= 0; --i)
        {
            //std::cout << "numPoses: "<< numPoses << std::endl;
            if (timeLaserInfoCur - cloudKeyPoses6D->points[i].time < 5.0)
                surroundingKeyPosesForPlaneDS->push_back(cloudKeyPoses3D->points[i]);
            else
                break;
        }
        extractPlaneCloud(surroundingKeyPosesForPlaneDS);
    }

    void extractPlaneCloud(pcl::PointCloud<PointXYZIRT>::Ptr cloudToExtract)
    {
        laserCloudPlaneFromMap->clear();
        int noPlaneCount = 0;//过往的帧中没有提取地面点帧的数量
        int farScanCount = 0;//距离较远的帧的数量
        std::cout << "surroundingKeyPosesForPlaneDS.size(): " << (int)cloudToExtract->size() << std::endl;
        for (int i = 0; i < (int)cloudToExtract->size(); ++i)
        {
            if (pointDistance(cloudToExtract->points[i], cloudKeyPoses3D->back()) > surroundingKeyframeSearchRadiusForPlane){
                farScanCount++;
                continue;
            }

            int thisKeyInd = (int)cloudToExtract->points[i].ring;
            if(thisKeyInd == 0){
                noPlaneCount++;
                continue;
            }
            int thisPlaneInd = thisKeyInd - (int)cloudToExtract->points[i].intensity;
            thisKeyInd = (int)cloudToExtract->points[i].intensity;
            std::cout << "********selected key frame id: " << thisKeyInd << "**********" << std::endl;
            std::cout << "********thisPlaneInd: " << thisPlaneInd << "**********" << std::endl;
            if (laserCloudPlaneMapContainer.find(thisKeyInd) != laserCloudPlaneMapContainer.end()) 
            {
                // transformed cloud available
                *laserCloudPlaneFromMap += laserCloudPlaneMapContainer[thisKeyInd];
            } else {
                {
                std::cout << "enter transform ground points" << std::endl;
                // transformed cloud not available
                std::cout << "planeCloudKeyFrames.size(): " << planeCloudKeyFrames.size() << std::endl;
                std::cout << "cloudKeyPoses6D->size(): " << cloudKeyPoses6D->size() << std::endl;

                std::cout << "before transform to world, ground points[0] is: " << "(" << (&(planeCloudKeyFrames[thisPlaneInd - 1]->points[0]))->x << ", "
                                                                                                                                    <<(&(planeCloudKeyFrames[thisPlaneInd - 1]->points[0]))->y << ", "
                                                                                                                                    <<(&(planeCloudKeyFrames[thisPlaneInd - 1]->points[0]))->z << ")"<< std::endl;
                std::cout << "cloudKeyPoses6D->points[thisKeyInd]: " << "(" << (&cloudKeyPoses6D->points[thisKeyInd])->x << ", "
                                                                                                                                << (&cloudKeyPoses6D->points[thisKeyInd])->y << ", "
                                                                                                                                << (&cloudKeyPoses6D->points[thisKeyInd])->z << ", "
                                                                                                                                << (&cloudKeyPoses6D->points[thisKeyInd])->roll << ", "
                                                                                                                                << (&cloudKeyPoses6D->points[thisKeyInd])->pitch << ", "
                                                                                                                                << (&cloudKeyPoses6D->points[thisKeyInd])->yaw << ")" << std::endl;            
                std::cout << "(planeCloudKeyFrames[thisPlaneInd - 1])->size(): " << (planeCloudKeyFrames[thisPlaneInd - 1])->size() << std::endl;     
                }                                                                                                    
                pcl::PointCloud<PointType> laserCloudPlaneTemp = *transformPointCloud(planeCloudKeyFrames[thisPlaneInd - 1],  &cloudKeyPoses6D->points[thisKeyInd]);
                std::cout << "after transform to world, ground points[0] is: " << "(" << laserCloudPlaneTemp.points[0].x << ", " 
                                                                                                                                << laserCloudPlaneTemp.points[0].y << ", "
                                                                                                                                << laserCloudPlaneTemp.points[0].z << ")" << std::endl;
                *laserCloudPlaneFromMap += laserCloudPlaneTemp;
                laserCloudPlaneMapContainer[thisKeyInd] = laserCloudPlaneTemp;
            }
        }
        std::cout <<"noPlaneCount: " << noPlaneCount << std::endl;
        std::cout <<"farScanCount: " << farScanCount << std::endl;
        if(noPlaneCount == ((int)cloudToExtract->size() - farScanCount)){
            flagForSubPlane = false;
        }else{
            flagForSubPlane = true;
        }

        if (laserCloudPlaneMapContainer.size() > 1000)
            laserCloudPlaneMapContainer.clear();
    }

    void segmentPlane(const lio_sam::cloud_infoConstPtr& msgIn)
    {
            // check cloud msg fields.
        static bool need_check_fields = true;
        static bool ring_field_exists = false;
        static bool time_field_exists = false;
        if (need_check_fields) {
            for (auto& field : msgIn->cloud_deskewed.fields)
            {
                if (field.name == "ring")
                {
                    ring_field_exists = true;
                    std::cout << "########## Congrats, `ring` field exists!" << std::endl;
                    continue;
                }
                if (field.name == "time" || field.name == "t")
                {
                    time_field_exists = true;
                    std::cout << "########## Congrats, `time` field exists!" << std::endl;
                    continue;
                }

            }
            if (!ring_field_exists) {
                std::cout << "########## Could not found `ring` field. " << std::endl;
            }
            if (!time_field_exists) {
                std::cout << "########## Could not found `time` field. " << std::endl;
            }
            need_check_fields = false;
        }

        clock_t time_start = clock();
    
        // Run ground-segmentation algorithm.
        if (ring_field_exists) {
            pcl::fromROSMsg(msgIn->cloud_deskewed, *custom_original_cloud_); 
            efficient_sgmtt_.Segment(custom_original_cloud_, &labels_, true);
        }
        else {
            pcl::fromROSMsg(msgIn->cloud_deskewed, *common_original_cloud_); 
            efficient_sgmtt_.Segment(common_original_cloud_, &labels_, true);
        }

        //std::cout << "common_original_cloud_.size() = " << common_original_cloud_->size() << std::endl;

        for(int i = 0; i < (*common_original_cloud_).size(); ++i){
            if(common_original_cloud_->points[i].intensity == 100){
                ground_points->push_back(common_original_cloud_->points[i]);
            }
        }

        /*
        for(int i = 10000; i < 10030; ++i){
            std::cout << "cout 20 ground points for debug: "<< ground_points->points[i] << std::endl;
        }
        */
        std::cout << "ground_points.size() = " << ground_points->size() << std::endl;
        //计算平面法向量*************************** 平均花费时长约为1.7ms
        {
        clock_t time_plane_vector_start = clock();
	    Eigen::Vector4f zx;//中心 齐次坐标 第4维是1
	    pcl::compute3DCentroid(*ground_points, zx); //xyz1
        std::cout << "centor point: " << "(" << zx(0) << ", " << zx(1) << ", " << zx(2) << ", " << zx(3) <<  ")" << std::endl;
	    Eigen::Matrix3f xiefancha;
	    pcl::computeCovarianceMatrixNormalized(*ground_points, zx, xiefancha);//计算归一化协方差矩阵
        std::cout << "CovarianceMatrixNormalized: " << std::endl << xiefancha << std::endl;
	    //计算特征值和特征向量
	    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solve(xiefancha, Eigen::ComputeEigenvectors);
	    Eigen::Vector3f values = solve.eigenvalues();//特征值
	    Eigen::Matrix3f vectors = solve.eigenvectors();//特征向量 列向量为特征向量 其中第一列为最小特征值对应的特征向量
        clock_t time_plane_vector_end = clock();
        float time_plane_vector = (float(time_plane_vector_end - time_plane_vector_start))/CLOCKS_PER_SEC;
        std::cout << "calculate plane vector take: " << time_plane_vector * 1000<< "ms" << std::endl;
	    //平面的法向量
	    double V_x = vectors(0, 0);
	    double V_y = vectors(1, 0);
	    double V_z = vectors(2, 0);
        double x0 = zx(0);
        double y0 = zx(1);
        double z0 = zx(2);
        double d = - (V_x * x0 + V_y * y0 + V_z * z0); 
        std::cout << "plane normal vector: " << "(" << V_x << ", " << V_y << ", " << V_z << ", " << d << ")" << std::endl;
        if(V_z < 0){
            Eigen::Vector4d currentPlane;
            currentPlane << -V_x, -V_y, -V_z, -d;
            planeCurrentQueue.push_back(currentPlane);
        }else{
            Eigen::Vector4d currentPlane;
            currentPlane << V_x, V_y, V_z, d;
            planeCurrentQueue.push_back(currentPlane);
        }


        }
        //**************************************************************************
         // Publish robot tf.
        {
            static tf::StampedTransform tf_msg;
            static tf::TransformBroadcaster tf_broadcaster;
            tf_msg.stamp_ = msgIn->cloud_deskewed.header.stamp;
            tf_msg.frame_id_ = base_link_frame_id;
            tf_msg.child_frame_id_ = sensor_frame_id;
            Eigen::Quaternionf rot_quat(params.kBaseToSensor.rotation());
            Eigen::Vector3f trans = params.kBaseToSensor.translation();
            tf_msg.setRotation(tf::Quaternion(rot_quat.x(), rot_quat.y(), rot_quat.z(), rot_quat.w()));//Tpc p: parent, c: child
            tf_msg.setOrigin(tf::Vector3(trans.x(), trans.y(), trans.z()));
            tf_broadcaster.sendTransform(tf_msg);
        }

        // Visualize results.
        if (segmted_cloud_publisher.getNumSubscribers() != 0){
            sensor_msgs::PointCloud2 cloudMsgTemp;
            if (common_original_cloud_->size()>0) {
                pcl::toROSMsg(*common_original_cloud_, cloudMsgTemp);
            }
            else {
                pcl::toROSMsg(*custom_original_cloud_, cloudMsgTemp);
            }
            cloudMsgTemp.header = msgIn->cloud_deskewed.header;
            cloudMsgTemp.header.frame_id = base_link_frame_id;
            segmted_cloud_publisher.publish(cloudMsgTemp);
        }

        if (transformed_cloud_publisher.getNumSubscribers() != 0){
            sensor_msgs::PointCloud2 cloudMsgTemp;
            if (ring_field_exists) {
                pcl::toROSMsg(*efficient_sgmtt_.GetTransformedCustomCloud(), cloudMsgTemp);
            }
            else {
                pcl::toROSMsg(*efficient_sgmtt_.GetTransformedCommonCloud(), cloudMsgTemp);
            }
            cloudMsgTemp.header = msgIn->cloud_deskewed.header;
            cloudMsgTemp.header.frame_id = base_link_frame_id;
            transformed_cloud_publisher.publish(cloudMsgTemp);
        }

        if (range_image_publisher.getNumSubscribers() != 0){
            cv::Mat imgTmp1, imgTmp2;
            imgTmp1 = efficient_sgmtt_.GetRangeImage();
            cv::normalize(imgTmp1,imgTmp2, 255, 0, cv::NORM_MINMAX);
            imgTmp2.convertTo(imgTmp1, CV_8UC1); // from CV_32FC1 to CV_8UC1
            cv::flip(imgTmp1, imgTmp2, 0);
            cv::applyColorMap(imgTmp2, imgTmp1, cv::COLORMAP_RAINBOW);
            cv::resize(imgTmp1, imgTmp2, cv::Size(imgTmp1.cols/3,imgTmp1.rows));
            sensor_msgs::ImagePtr imgMsgTemp = 
                cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgTmp2).toImageMsg();
            range_image_publisher.publish(*imgMsgTemp);
        }

        if (extracted_lines_publisher.getNumSubscribers() != 0) {
            visualization_msgs::MarkerArray lines_array;
            const auto& geometry_lines = efficient_sgmtt_.GetExtractedLines();
            if (!geometry_lines.empty()) {
                float kEdgeScale = 0.05;
                visualization_msgs::Marker edge;
                edge.header = msgIn->cloud_deskewed.header;
                edge.header.frame_id = base_link_frame_id;
                edge.action = visualization_msgs::Marker::ADD;
                edge.ns = "EOS_lines";
                edge.id = 0;
                edge.type = visualization_msgs::Marker::LINE_STRIP;
                edge.scale.x = kEdgeScale;
                edge.scale.y = kEdgeScale;
                edge.scale.z = kEdgeScale;
                edge.color.r = 0.0;
                edge.color.g = 1.0;
                edge.color.b = 1.0;
                edge.color.a = 1.0;
                geometry_msgs::Point pStart;
                geometry_msgs::Point pEnd;
                int id=0;
                for(const auto& curr_line : geometry_lines)
                {
                    //遍历每个边 将位置赋值
                    edge.points.clear();
                    edge.id = id;
                    pStart.x = curr_line.start_point.x;
                    pStart.y = curr_line.start_point.y;
                    pStart.z = curr_line.start_point.z;
                    edge.points.push_back(pStart);
                    pEnd.x = curr_line.end_point.x;
                    pEnd.y = curr_line.end_point.y;
                    pEnd.z = curr_line.end_point.z;
                    edge.points.push_back(pEnd);
                    if (curr_line.label==LineLabel::GROUND) {
                        edge.scale.x = kEdgeScale;
                        edge.scale.y = kEdgeScale;
                        edge.scale.z = kEdgeScale;
                        edge.color.r = 1.0;
                        edge.color.g = 0.8;
                        edge.color.b = 0.0;
                    }
                    else if (curr_line.label==LineLabel::WALL) {
                        edge.scale.x = 2*kEdgeScale;
                        edge.scale.y = 2*kEdgeScale;
                        edge.scale.z = 2*kEdgeScale;
                        edge.color.r = 0.0;
                        edge.color.g = 1.0;
                        edge.color.b = 0.0;
                    }
                    lines_array.markers.push_back(visualization_msgs::Marker(edge));
                    id++;
                }
            }
            extracted_lines_publisher.publish(lines_array);
        }
    }

    void gpsHandler(const nav_msgs::Odometry::ConstPtr& gpsMsg)
    {
        gpsQueue.push_back(*gpsMsg);
        Eigen::Matrix3d base_R_gps;
        Eigen::Quaterniond base_q_gps;
        Eigen::Vector3d base_t_gps;
        base_q_gps.x() = (*gpsMsg).pose.pose.orientation.x;
        base_q_gps.y() = (*gpsMsg).pose.pose.orientation.y;
        base_q_gps.z() = (*gpsMsg).pose.pose.orientation.z;
        base_q_gps.w() = (*gpsMsg).pose.pose.orientation.w;
        base_t_gps(0) = (*gpsMsg).pose.pose.position.x;
        base_t_gps(1) = (*gpsMsg).pose.pose.position.y;
        base_t_gps(2) = (*gpsMsg).pose.pose.position.z;
        base_R_gps = base_q_gps.normalized().toRotationMatrix();

        std::ofstream outputGPS_tum(savePCDDirectory + "gps_tum.txt",  std::ios::app);
        std::ofstream outputGPS_kitti(savePCDDirectory + "gps_kitti.txt",  std::ios::app);
        outputGPS_tum.setf(std::ios::scientific, std::ios::floatfield);
        outputGPS_tum.precision(12);
        outputGPS_tum << (*gpsMsg).header.stamp.toSec() << " " << base_t_gps(0) << " " << base_t_gps(1) << " " << base_t_gps(2) << " "
                       <<  base_q_gps.x() << " " << base_q_gps.y() << " " << base_q_gps.z() << " " << base_q_gps.w() << endl;
        outputGPS_kitti.setf(std::ios::scientific, std::ios::floatfield);
        outputGPS_kitti.precision(6);
        outputGPS_kitti << base_R_gps(0, 0) << " " << base_R_gps(0, 1) << " " << base_R_gps(0, 2) << " " << base_t_gps(0) << " "
               << base_R_gps(1, 0) << " " << base_R_gps(1, 1) << " " << base_R_gps(1, 2) << " " << base_t_gps(1) << " "
               << base_R_gps(2, 0) << " " << base_R_gps(2, 1) << " " << base_R_gps(2, 2) << " " << base_t_gps(2) << std::endl;
    }

    void pointAssociateToMap(PointType const * const pi, PointType * const po)
    {
        po->x = transPointAssociateToMap(0,0) * pi->x + transPointAssociateToMap(0,1) * pi->y + transPointAssociateToMap(0,2) * pi->z + transPointAssociateToMap(0,3);
        po->y = transPointAssociateToMap(1,0) * pi->x + transPointAssociateToMap(1,1) * pi->y + transPointAssociateToMap(1,2) * pi->z + transPointAssociateToMap(1,3);
        po->z = transPointAssociateToMap(2,0) * pi->x + transPointAssociateToMap(2,1) * pi->y + transPointAssociateToMap(2,2) * pi->z + transPointAssociateToMap(2,3);
        po->intensity = pi->intensity;
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        PointType *pointFrom;

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
        
        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cloudSize; ++i)
        {
            pointFrom = &cloudIn->points[i];
            cloudOut->points[i].x = transCur(0,0) * pointFrom->x + transCur(0,1) * pointFrom->y + transCur(0,2) * pointFrom->z + transCur(0,3);
            cloudOut->points[i].y = transCur(1,0) * pointFrom->x + transCur(1,1) * pointFrom->y + transCur(1,2) * pointFrom->z + transCur(1,3);
            cloudOut->points[i].z = transCur(2,0) * pointFrom->x + transCur(2,1) * pointFrom->y + transCur(2,2) * pointFrom->z + transCur(2,3);
            cloudOut->points[i].intensity = pointFrom->intensity;
        }
        return cloudOut;
    }

    gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                                  gtsam::Point3(double(thisPoint.x),    double(thisPoint.y),     double(thisPoint.z)));
    }

    gtsam::Pose3 trans2gtsamPose(float transformIn[])
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]), 
                                  gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
    }

    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
    { 
        return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
    }

    Eigen::Affine3f trans2Affine3f(float transformIn[])
    {
        return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
    }

    PointTypePose trans2PointTypePose(float transformIn[])
    {
        PointTypePose thisPose6D;
        thisPose6D.x = transformIn[3];
        thisPose6D.y = transformIn[4];
        thisPose6D.z = transformIn[5];
        thisPose6D.roll  = transformIn[0];
        thisPose6D.pitch = transformIn[1];
        thisPose6D.yaw   = transformIn[2];
        return thisPose6D;
    }


    void visualizeGlobalMapThread()
    {
        //
        ros::Rate rate(0.2);
        while (ros::ok()){
            rate.sleep();
            publishGlobalMap();
        }

        if (savePCD == false)
            return;

        // save pose graph (runs when programe is closing)
        cout << "****************************************************" << endl; 
        cout << "Saving the posegraph ..." << endl; // giseop

        for(auto& _line: vertices_str)
            pgSaveStream << _line << std::endl;
        for(auto& _line: edges_str)
            pgSaveStream << _line << std::endl;

        pgSaveStream.close();
        // pgVertexSaveStream.close();
        // pgEdgeSaveStream.close();

        const std::string kitti_format_pg_filename {savePCDDirectory + "optimized_poses.txt"};
        saveOptimizedVerticesKITTIformat(isamCurrentEstimate, kitti_format_pg_filename);

        // save map 
        cout << "****************************************************" << endl;
        cout << "Saving map to pcd files ..." << endl;
        // save key frame transformations
        pcl::io::savePCDFileASCII(savePCDDirectory + "trajectory.pcd", *cloudKeyPoses3D);
        pcl::io::savePCDFileASCII(savePCDDirectory + "transformations.pcd", *cloudKeyPoses6D);
        // extract global point cloud map        
        pcl::PointCloud<PointType>::Ptr globalCornerCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalCornerCloudDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalSurfCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalSurfCloudDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());
        //for plane
        pcl::PointCloud<PointType>::Ptr globalPlaneCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalPlaneCloudDS(new pcl::PointCloud<PointType>());
        for(int i = 0; i < (int)planeCloudKeyFrames.size(); i++){
            //j表示该地面帧在全局中的索引值
            int j = planeCloudKeyFrames[i]->points[0].intensity;
            *globalPlaneCloud += *transformPointCloud(planeCloudKeyFrames[i], &cloudKeyPoses6D->points[j]);
        }
        for (int i = 0; i < (int)cloudKeyPoses3D->size(); i++) {
            *globalCornerCloud += *transformPointCloud(cornerCloudKeyFrames[i],  &cloudKeyPoses6D->points[i]);
            *globalSurfCloud   += *transformPointCloud(surfCloudKeyFrames[i],    &cloudKeyPoses6D->points[i]);
            cout << "\r" << std::flush << "Processing feature cloud " << i << " of " << cloudKeyPoses6D->size() << " ...";
        }
        
        //对地面点降采样并保存
        downSizeFilterPlane.setInputCloud(globalPlaneCloud);
        downSizeFilterPlane.filter(*globalPlaneCloudDS);
        pcl::io::savePCDFileASCII(savePCDDirectory + "cloudPlane.pcd", *globalPlaneCloudDS);
        // down-sample and save corner cloud
        downSizeFilterCorner.setInputCloud(globalCornerCloud);
        downSizeFilterCorner.filter(*globalCornerCloudDS);
        pcl::io::savePCDFileASCII(savePCDDirectory + "cloudCorner.pcd", *globalCornerCloudDS);
        // down-sample and save surf cloud
        downSizeFilterSurf.setInputCloud(globalSurfCloud);
        downSizeFilterSurf.filter(*globalSurfCloudDS);
        pcl::io::savePCDFileASCII(savePCDDirectory + "cloudSurf.pcd", *globalSurfCloudDS);
        // down-sample and save global point cloud map
        *globalMapCloud += *globalCornerCloud;
        *globalMapCloud += *globalSurfCloud;
        pcl::io::savePCDFileASCII(savePCDDirectory + "cloudGlobal.pcd", *globalMapCloud);
        cout << "****************************************************" << endl;
        cout << "Saving map to pcd files completed" << endl;
    }

    void publishGlobalMap()
    {
        if (pubLaserCloudSurround.getNumSubscribers() == 0)
            return;

        if (cloudKeyPoses3D->points.empty() == true)
            return;

        pcl::KdTreeFLANN<PointXYZIRT>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointXYZIRT>());;
        pcl::PointCloud<PointXYZIRT>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointXYZIRT>());
        pcl::PointCloud<PointXYZIRT>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointXYZIRT>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointType>());

        // kd-tree to find near key frames to visualize
        std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;
        // search near key frames to visualize
        mtx.lock();
        kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
        kdtreeGlobalMap->radiusSearch(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
        mtx.unlock();

        for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
            globalMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
        // downsample near selected key frames
        pcl::VoxelGrid<PointXYZIRT> downSizeFilterGlobalMapKeyPoses; // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity); // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
        downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);

        // extract visualized and downsampled key frames
        for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i){
            if (pointDistance(globalMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) > globalMapVisualizationSearchRadius)
                continue;
            int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
            *globalMapKeyFrames += *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],  &cloudKeyPoses6D->points[thisKeyInd]);
            *globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
        }
        // downsample visualized points
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames; // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize); // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
        downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
        publishCloud(&pubLaserCloudSurround, globalMapKeyFramesDS, timeLaserInfoStamp, odometryFrame);
    }


    void loopClosureThread()
    {
        if (loopClosureEnableFlag == false)
            return;

        ros::Rate rate(loopClosureFrequency);
        while (ros::ok())
        {
            rate.sleep();
            performRSLoopClosure();
            performSCLoopClosure(); // giseop
            visualizeLoopClosure();
        }
    }

    void loopInfoHandler(const std_msgs::Float64MultiArray::ConstPtr& loopMsg)
    {
        std::lock_guard<std::mutex> lock(mtxLoopInfo);
        if (loopMsg->data.size() != 2)
            return;

        loopInfoVec.push_back(*loopMsg);

        while (loopInfoVec.size() > 5)
            loopInfoVec.pop_front();
    }

    void performRSLoopClosure()
    {
        if (cloudKeyPoses3D->points.empty() == true)
            return;

        mtx.lock();
        *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
        copy_cloudKeyPoses2D->clear(); // giseop
        *copy_cloudKeyPoses2D = *cloudKeyPoses3D; // giseop 
        *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
        mtx.unlock();

        // find keys
        int loopKeyCur;
        int loopKeyPre;
        if (detectLoopClosureExternal(&loopKeyCur, &loopKeyPre) == false)
            if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false)
                return;

        std::cout << "RS loop found! between " << loopKeyCur << " and " << loopKeyPre << "." << std::endl; // giseop

        // extract cloud
        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
        {
            loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);
            loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum);//前后各25帧
            if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
                return;
            if (pubHistoryKeyFrames.getNumSubscribers() != 0)
                publishCloud(&pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
        }

        // ICP Settings
        static pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(150); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter 
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        // Align clouds
        icp.setInputSource(cureKeyframeCloud);
        icp.setInputTarget(prevKeyframeCloud);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result);

        if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore) {
            std::cout << "ICP fitness test failed (" << icp.getFitnessScore() << " > " << historyKeyframeFitnessScore << "). Reject this RS loop." << std::endl;
            return;
        } else {
            std::cout << "ICP fitness test passed (" << icp.getFitnessScore() << " < " << historyKeyframeFitnessScore << "). Add this RS loop." << std::endl;
        }

        // publish corrected cloud
        if (pubIcpKeyFrames.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
            publishCloud(&pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
        }

        // Get pose transformation
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = icp.getFinalTransformation();
        // transform from world origin to wrong pose Twc
        Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
        // transform from world origin to corrected pose Twp = Twc * Tcp
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;// pre-multiplying -> successive rotation about a fixed frame
        pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
        gtsam::Vector Vector6(6);
        float noiseScore = icp.getFitnessScore();
        Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);

        // Add pose constraint
        mtx.lock();
        loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
        loopPoseQueue.push_back(poseFrom.between(poseTo));
        loopNoiseQueue.push_back(constraintNoise);
        mtx.unlock();

        // add loop constriant
        // loopIndexContainer[loopKeyCur] = loopKeyPre;
        loopIndexContainer.insert(std::pair<int, int>(loopKeyCur, loopKeyPre)); // giseop for multimap
    } // performRSLoopClosure


    void performSCLoopClosure()
    {
        if (cloudKeyPoses3D->points.empty() == true)
            return;

        // find keys
        auto detectResult = scManager.detectLoopClosureID(); // first: nn index, second: yaw diff 
        int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;;
        int loopKeyPre = detectResult.first;
        float yawDiffRad = detectResult.second; // not use for v1 (because pcl icp withi initial somthing wrong...)
        if( loopKeyPre == -1 /* No loop found */)
            return;

        std::cout << "SC loop found! between " << loopKeyCur << " and " << loopKeyPre << "." << std::endl; // giseop

        // extract cloud
        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
        {
            // loopFindNearKeyframesWithRespectTo(cureKeyframeCloud, loopKeyCur, 0, loopKeyPre); // giseop 
            // loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum);

            int base_key = 0;
            loopFindNearKeyframesWithRespectTo(cureKeyframeCloud, loopKeyCur, 0, base_key); // giseop 
            loopFindNearKeyframesWithRespectTo(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum, base_key); // giseop 

            if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
                return;
            if (pubHistoryKeyFrames.getNumSubscribers() != 0)
                publishCloud(&pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
        }

        // ICP Settings
        static pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(150); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter 
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        // Align clouds
        icp.setInputSource(cureKeyframeCloud);
        icp.setInputTarget(prevKeyframeCloud);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result);
        // giseop 
        // TODO icp align with initial 

        if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore) {
            std::cout << "ICP fitness test failed (" << icp.getFitnessScore() << " > " << historyKeyframeFitnessScore << "). Reject this SC loop." << std::endl;
            return;
        } else {
            std::cout << "ICP fitness test passed (" << icp.getFitnessScore() << " < " << historyKeyframeFitnessScore << "). Add this SC loop." << std::endl;
        }

        // publish corrected cloud
        if (pubIcpKeyFrames.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
            publishCloud(&pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
        }

        // Get pose transformation
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = icp.getFinalTransformation();

        // // transform from world origin to wrong pose
        // Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
        // // transform from world origin to corrected pose
        // Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;// pre-multiplying -> successive rotation about a fixed frame
        // pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
        // gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        // gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);

        // gtsam::Vector Vector6(6);
        // float noiseScore = icp.getFitnessScore();
        // Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        // noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);

        // giseop 
        pcl::getTranslationAndEulerAngles (correctionLidarFrame, x, y, z, roll, pitch, yaw);
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));

        // giseop, robust kernel for a SC loop
        float robustNoiseScore = 0.5; // constant is ok...
        gtsam::Vector robustNoiseVector6(6); 
        robustNoiseVector6 << robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore;
        noiseModel::Base::shared_ptr robustConstraintNoise; 
        robustConstraintNoise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure, but with a good front-end loop detector, Cauchy is empirically enough.
            gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6)
        ); // - checked it works. but with robust kernel, map modification may be delayed (i.e,. requires more true-positive loop factors)

        // Add pose constraint
        mtx.lock();
        loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
        loopPoseQueue.push_back(poseFrom.between(poseTo));
        loopNoiseQueue.push_back(robustConstraintNoise);
        mtx.unlock();

        // add loop constriant
        // loopIndexContainer[loopKeyCur] = loopKeyPre;
        loopIndexContainer.insert(std::pair<int, int>(loopKeyCur, loopKeyPre)); // giseop for multimap
    } // performSCLoopClosure


    bool detectLoopClosureDistance(int *latestID, int *closestID)
    {
        int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
        int loopKeyPre = -1;

        // check loop constraint added before
        auto it = loopIndexContainer.find(loopKeyCur);
        if (it != loopIndexContainer.end())
            return false;

        // find the closest history key frame
        std::vector<int> pointSearchIndLoop;
        std::vector<float> pointSearchSqDisLoop; // unused 
        // kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D);
        // kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->back(), historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);
 
        for (int i = 0; i < (int)copy_cloudKeyPoses2D->size(); i++) // giseop
            copy_cloudKeyPoses2D->points[i].z = 1.1; // to relieve the z-axis drift, 1.1 is just foo val

        kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses2D); // giseop
        kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses2D->back(), historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0); // giseop
        
        // std::cout << "the number of RS-loop candidates  " << pointSearchIndLoop.size() << "." << std::endl; // giseop
        for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i)
        {
            int id = pointSearchIndLoop[i];
            if (abs(copy_cloudKeyPoses6D->points[id].time - timeLaserInfoCur) > historyKeyframeSearchTimeDiff)
            {
                loopKeyPre = id;
                break;
            }
        }

        if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
            return false;

        *latestID = loopKeyCur;
        *closestID = loopKeyPre;

        return true;
    }

    bool detectLoopClosureExternal(int *latestID, int *closestID)
    {
        // this function is not used yet, please ignore it
        int loopKeyCur = -1;
        int loopKeyPre = -1;

        std::lock_guard<std::mutex> lock(mtxLoopInfo);
        if (loopInfoVec.empty())
            return false;

        double loopTimeCur = loopInfoVec.front().data[0];
        double loopTimePre = loopInfoVec.front().data[1];
        loopInfoVec.pop_front();

        if (abs(loopTimeCur - loopTimePre) < historyKeyframeSearchTimeDiff)
            return false;

        int cloudSize = copy_cloudKeyPoses6D->size();
        if (cloudSize < 2)
            return false;

        // latest key
        loopKeyCur = cloudSize - 1;
        for (int i = cloudSize - 1; i >= 0; --i)
        {
            if (copy_cloudKeyPoses6D->points[i].time >= loopTimeCur)
                loopKeyCur = round(copy_cloudKeyPoses6D->points[i].intensity);
            else
                break;
        }

        // previous key
        loopKeyPre = 0;
        for (int i = 0; i < cloudSize; ++i)
        {
            if (copy_cloudKeyPoses6D->points[i].time <= loopTimePre)
                loopKeyPre = round(copy_cloudKeyPoses6D->points[i].intensity);
            else
                break;
        }

        if (loopKeyCur == loopKeyPre)
            return false;

        auto it = loopIndexContainer.find(loopKeyCur);
        if (it != loopIndexContainer.end())
            return false;

        *latestID = loopKeyCur;
        *closestID = loopKeyPre;

        return true;
    }

    void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum)
    {
        // extract near keyframes
        nearKeyframes->clear();
        int cloudSize = copy_cloudKeyPoses6D->size();
        for (int i = -searchNum; i <= searchNum; ++i)
        {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= cloudSize )
                continue;
            *nearKeyframes += *transformPointCloud(cornerCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[keyNear]);
            *nearKeyframes += *transformPointCloud(surfCloudKeyFrames[keyNear],   &copy_cloudKeyPoses6D->points[keyNear]);
        }

        if (nearKeyframes->empty())
            return;

        // downsample near keyframes
        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        downSizeFilterICP.setInputCloud(nearKeyframes);
        downSizeFilterICP.filter(*cloud_temp);
        *nearKeyframes = *cloud_temp;
    }

    void loopFindNearKeyframesWithRespectTo(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum, const int _wrt_key)
    {
        // extract near keyframes
        nearKeyframes->clear();
        int cloudSize = copy_cloudKeyPoses6D->size();
        for (int i = -searchNum; i <= searchNum; ++i)
        {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= cloudSize )
                continue;
            *nearKeyframes += *transformPointCloud(cornerCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[_wrt_key]);
            *nearKeyframes += *transformPointCloud(surfCloudKeyFrames[keyNear],   &copy_cloudKeyPoses6D->points[_wrt_key]);
        }

        if (nearKeyframes->empty())
            return;

        // downsample near keyframes
        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        downSizeFilterICP.setInputCloud(nearKeyframes);
        downSizeFilterICP.filter(*cloud_temp);
        *nearKeyframes = *cloud_temp;
    }

    void visualizeLoopClosure()
    {
        visualization_msgs::MarkerArray markerArray;
        // loop nodes
        visualization_msgs::Marker markerNode;
        markerNode.header.frame_id = odometryFrame;
        markerNode.header.stamp = timeLaserInfoStamp;
        markerNode.action = visualization_msgs::Marker::ADD;
        markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
        markerNode.ns = "loop_nodes";
        markerNode.id = 0;
        markerNode.pose.orientation.w = 1;
        markerNode.scale.x = 0.3; markerNode.scale.y = 0.3; markerNode.scale.z = 0.3; 
        markerNode.color.r = 0; markerNode.color.g = 0.8; markerNode.color.b = 1;
        markerNode.color.a = 1;
        // loop edges
        visualization_msgs::Marker markerEdge;
        markerEdge.header.frame_id = odometryFrame;
        markerEdge.header.stamp = timeLaserInfoStamp;
        markerEdge.action = visualization_msgs::Marker::ADD;
        markerEdge.type = visualization_msgs::Marker::LINE_LIST;
        markerEdge.ns = "loop_edges";
        markerEdge.id = 1;
        markerEdge.pose.orientation.w = 1;
        markerEdge.scale.x = 0.1; markerEdge.scale.y = 0.1; markerEdge.scale.z = 0.1;
        markerEdge.color.r = 0.9; markerEdge.color.g = 0.9; markerEdge.color.b = 0;
        markerEdge.color.a = 1;

        for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it)
        {
            int key_cur = it->first;
            int key_pre = it->second;
            geometry_msgs::Point p;
            p.x = copy_cloudKeyPoses6D->points[key_cur].x;
            p.y = copy_cloudKeyPoses6D->points[key_cur].y;
            p.z = copy_cloudKeyPoses6D->points[key_cur].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
            p.x = copy_cloudKeyPoses6D->points[key_pre].x;
            p.y = copy_cloudKeyPoses6D->points[key_pre].y;
            p.z = copy_cloudKeyPoses6D->points[key_pre].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
        }

        markerArray.markers.push_back(markerNode);
        markerArray.markers.push_back(markerEdge);
        pubLoopConstraintEdge.publish(markerArray);
    }



    void updateInitialGuess()
    {
        //全局第一帧时，此时transformTobeMapped的rpy xyz都为0
        // save current transformation before any processing
        incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);

        static Eigen::Affine3f lastImuTransformation;
        // initialization 当为全局第一帧时，位姿的初值采用imu的roll、pitch（根据重力对齐是可观的），yaw（还是不可观，可用可不用）
        // 位置xyz设置为0，表示初始帧坐标系的原点与世界坐标系的原点重合
        if (cloudKeyPoses3D->points.empty())
        {
            transformTobeMapped[0] = cloudInfo.imuRollInit;
            transformTobeMapped[1] = cloudInfo.imuPitchInit;
            transformTobeMapped[2] = cloudInfo.imuYawInit;

            if (!useImuHeadingInitialization)
                transformTobeMapped[2] = 0;

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
            return;
        }

        // use imu pre-integration estimation for pose guess
        static bool lastImuPreTransAvailable = false;
        static Eigen::Affine3f lastImuPreTransformation;
        if (cloudInfo.odomAvailable == true)
        {
            Eigen::Affine3f transBack = pcl::getTransformation(cloudInfo.initialGuessX,    cloudInfo.initialGuessY,     cloudInfo.initialGuessZ, 
                                                               cloudInfo.initialGuessRoll, cloudInfo.initialGuessPitch, cloudInfo.initialGuessYaw);
            if (lastImuPreTransAvailable == false)
            {
                lastImuPreTransformation = transBack;
                lastImuPreTransAvailable = true;
            } else {
                Eigen::Affine3f transIncre = lastImuPreTransformation.inverse() * transBack;
                Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
                Eigen::Affine3f transFinal = transTobe * transIncre;
                pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                              transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

                lastImuPreTransformation = transBack;

                lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
                return;
            }
        }

        // use imu incremental estimation for pose guess (only rotation)
        if (cloudInfo.imuAvailable == true)
        {
            Eigen::Affine3f transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);
            Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;

            Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
            Eigen::Affine3f transFinal = transTobe * transIncre;
            pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                          transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
            return;
        }
    }

    void extractForLoopClosure()
    {
        pcl::PointCloud<PointXYZIRT>::Ptr cloudToExtract(new pcl::PointCloud<PointXYZIRT>());
        int numPoses = cloudKeyPoses3D->size();
        for (int i = numPoses-1; i >= 0; --i)
        {
            if ((int)cloudToExtract->size() <= surroundingKeyframeSize)
                cloudToExtract->push_back(cloudKeyPoses3D->points[i]);
            else
                break;
        }

        extractCloud(cloudToExtract);
    }

    void extractNearby()
    {
        pcl::PointCloud<PointXYZIRT>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointXYZIRT>());
        pcl::PointCloud<PointXYZIRT>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointXYZIRT>());
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        // extract all the nearby key poses and downsample them
        kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D); // create kd-tree
        kdtreeSurroundingKeyPoses->radiusSearch(cloudKeyPoses3D->back(), (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis);
        for (int i = 0; i < (int)pointSearchInd.size(); ++i)
        {
            int id = pointSearchInd[i];
            surroundingKeyPoses->push_back(cloudKeyPoses3D->points[id]);
        }

        downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
        downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);

        // also extract some latest key frames in case the robot rotates in one position
        int numPoses = cloudKeyPoses3D->size();
        for (int i = numPoses-1; i >= 0; --i)
        {
            if (timeLaserInfoCur - cloudKeyPoses6D->points[i].time < 10.0)
                surroundingKeyPosesDS->push_back(cloudKeyPoses3D->points[i]);
            else
                break;
        }

        extractCloud(surroundingKeyPosesDS);
    }

    void extractCloud(pcl::PointCloud<PointXYZIRT>::Ptr cloudToExtract)
    {
        // fuse the map
        laserCloudCornerFromMap->clear();
        laserCloudSurfFromMap->clear(); 
        for (int i = 0; i < (int)cloudToExtract->size(); ++i)
        {
            if (pointDistance(cloudToExtract->points[i], cloudKeyPoses3D->back()) > surroundingKeyframeSearchRadius)
                continue;

            int thisKeyInd = (int)cloudToExtract->points[i].intensity;
            if (laserCloudMapContainer.find(thisKeyInd) != laserCloudMapContainer.end()) 
            {
                // transformed cloud available
                *laserCloudCornerFromMap += laserCloudMapContainer[thisKeyInd].first;
                *laserCloudSurfFromMap   += laserCloudMapContainer[thisKeyInd].second;
            } else {
                // transformed cloud not available
                //这里为什么直接使用thisKeyInd而不用减1
                //答：因为intensity是指位姿点聚类中位姿点的数量 - 1
                pcl::PointCloud<PointType> laserCloudCornerTemp = *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],  &cloudKeyPoses6D->points[thisKeyInd]);
                pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
                *laserCloudCornerFromMap += laserCloudCornerTemp;
                *laserCloudSurfFromMap   += laserCloudSurfTemp;
                laserCloudMapContainer[thisKeyInd] = make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
            }
            
        }

        // Downsample the surrounding corner key frames (or map)
        downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
        downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
        laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->size();
        // Downsample the surrounding surf key frames (or map)
        downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
        downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
        laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();

        // clear map cache if too large
        if (laserCloudMapContainer.size() > 1000)//该map容器的作用就是空间换时间，这里清除则时间换空间
            laserCloudMapContainer.clear();
    }

    void extractSurroundingKeyFrames()
    {
        if (cloudKeyPoses3D->points.empty() == true)
            return; 
        
        // if (loopClosureEnableFlag == true)
        // {
        //     extractForLoopClosure();    
        // } else {
        //     extractNearby();
        // }

        extractNearby();
    }

    void downsampleCurrentScan()
    {
        // giseop
        laserCloudRawDS->clear();
        downSizeFilterSC.setInputCloud(laserCloudRaw);
        downSizeFilterSC.filter(*laserCloudRawDS);        

        // Downsample cloud from current scan
        laserCloudCornerLastDS->clear();
        downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
        downSizeFilterCorner.filter(*laserCloudCornerLastDS);
        laserCloudCornerLastDSNum = laserCloudCornerLastDS->size();

        laserCloudSurfLastDS->clear();
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurf.filter(*laserCloudSurfLastDS);
        laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();        

    }

    void updatePointAssociateToMap()
    {
        transPointAssociateToMap = trans2Affine3f(transformTobeMapped);
    }

    void cornerOptimization()
    {
        updatePointAssociateToMap();

        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < laserCloudCornerLastDSNum; i++)
        {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = laserCloudCornerLastDS->points[i];
            pointAssociateToMap(&pointOri, &pointSel);
            kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));
                    
            if (pointSearchSqDis[4] < 1.0) {
                float cx = 0, cy = 0, cz = 0;
                for (int j = 0; j < 5; j++) {
                    cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
                    cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
                    cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
                }
                cx /= 5; cy /= 5;  cz /= 5;

                float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
                for (int j = 0; j < 5; j++) {
                    float ax = laserCloudCornerFromMapDS->points[pointSearchInd[j]].x - cx;
                    float ay = laserCloudCornerFromMapDS->points[pointSearchInd[j]].y - cy;
                    float az = laserCloudCornerFromMapDS->points[pointSearchInd[j]].z - cz;

                    a11 += ax * ax; a12 += ax * ay; a13 += ax * az;
                    a22 += ay * ay; a23 += ay * az;
                    a33 += az * az;
                }
                a11 /= 5; a12 /= 5; a13 /= 5; a22 /= 5; a23 /= 5; a33 /= 5;

                matA1.at<float>(0, 0) = a11; matA1.at<float>(0, 1) = a12; matA1.at<float>(0, 2) = a13;
                matA1.at<float>(1, 0) = a12; matA1.at<float>(1, 1) = a22; matA1.at<float>(1, 2) = a23;
                matA1.at<float>(2, 0) = a13; matA1.at<float>(2, 1) = a23; matA1.at<float>(2, 2) = a33;

                cv::eigen(matA1, matD1, matV1);

                if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {

                    float x0 = pointSel.x;
                    float y0 = pointSel.y;
                    float z0 = pointSel.z;
                    float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                    float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                    float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                    float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                    float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                    float z2 = cz - 0.1 * matV1.at<float>(0, 2);

                    float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                                    + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                                    + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)) * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

                    float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

                    float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                              + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

                    float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                               - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                    float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                               + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                    float ld2 = a012 / l12;

                    float s = 1 - 0.9 * fabs(ld2);

                    coeff.x = s * la;
                    coeff.y = s * lb;
                    coeff.z = s * lc;
                    coeff.intensity = s * ld2;

                    if (s > 0.1) {
                        laserCloudOriCornerVec[i] = pointOri;
                        coeffSelCornerVec[i] = coeff;
                        laserCloudOriCornerFlag[i] = true;
                    }
                }
            }
        }
    }

    void surfOptimization()
    {
        updatePointAssociateToMap();

        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < laserCloudSurfLastDSNum; i++)
        {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = laserCloudSurfLastDS->points[i];
            pointAssociateToMap(&pointOri, &pointSel); 
            kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            Eigen::Matrix<float, 5, 3> matA0;
            Eigen::Matrix<float, 5, 1> matB0;
            Eigen::Vector3f matX0;

            matA0.setZero();
            matB0.fill(-1);
            matX0.setZero();

            if (pointSearchSqDis[4] < 1.0) {
                for (int j = 0; j < 5; j++) {
                    matA0(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                    matA0(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                    matA0(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
                }

                matX0 = matA0.colPivHouseholderQr().solve(matB0);

                float pa = matX0(0, 0);
                float pb = matX0(1, 0);
                float pc = matX0(2, 0);
                float pd = 1;

                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                pa /= ps; pb /= ps; pc /= ps; pd /= ps;

                bool planeValid = true;
                for (int j = 0; j < 5; j++) {
                    if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                             pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                             pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2) {
                        planeValid = false;
                        break;
                    }
                }

                if (planeValid) {
                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
                            + pointSel.y * pointSel.y + pointSel.z * pointSel.z));

                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;

                    if (s > 0.1) {
                        laserCloudOriSurfVec[i] = pointOri;
                        coeffSelSurfVec[i] = coeff;
                        laserCloudOriSurfFlag[i] = true;
                    }
                }
            }
        }
    }

    void combineOptimizationCoeffs()
    {
        // combine corner coeffs
        for (int i = 0; i < laserCloudCornerLastDSNum; ++i){
            if (laserCloudOriCornerFlag[i] == true){
                laserCloudOri->push_back(laserCloudOriCornerVec[i]);
                coeffSel->push_back(coeffSelCornerVec[i]);
            }
        }
        // combine surf coeffs
        for (int i = 0; i < laserCloudSurfLastDSNum; ++i){
            if (laserCloudOriSurfFlag[i] == true){
                laserCloudOri->push_back(laserCloudOriSurfVec[i]);
                coeffSel->push_back(coeffSelSurfVec[i]);
            }
        }
        // reset flag for next iteration
        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
    }

    bool LMOptimization(int iterCount)
    {
        // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
        // lidar <- camera      ---     camera <- lidar
        // x = z                ---     x = y
        // y = x                ---     y = z
        // z = y                ---     z = x
        // roll = yaw           ---     roll = pitch
        // pitch = roll         ---     pitch = yaw
        // yaw = pitch          ---     yaw = roll

        // lidar -> camera
        float srx = sin(transformTobeMapped[1]);
        float crx = cos(transformTobeMapped[1]);
        float sry = sin(transformTobeMapped[2]);
        float cry = cos(transformTobeMapped[2]);
        float srz = sin(transformTobeMapped[0]);
        float crz = cos(transformTobeMapped[0]);

        int laserCloudSelNum = laserCloudOri->size();
        if (laserCloudSelNum < 50) {
            return false;
        }

        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));

        PointType pointOri, coeff;

        for (int i = 0; i < laserCloudSelNum; i++) {
            // lidar -> camera
            pointOri.x = laserCloudOri->points[i].y;
            pointOri.y = laserCloudOri->points[i].z;
            pointOri.z = laserCloudOri->points[i].x;
            // lidar -> camera
            coeff.x = coeffSel->points[i].y;
            coeff.y = coeffSel->points[i].z;
            coeff.z = coeffSel->points[i].x;
            coeff.intensity = coeffSel->points[i].intensity;
            // in camera
            float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                      + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                      + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

            float ary = ((cry*srx*srz - crz*sry)*pointOri.x 
                      + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                      + ((-cry*crz - srx*sry*srz)*pointOri.x 
                      + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

            float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                      + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                      + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;
            // lidar -> camera
            matA.at<float>(i, 0) = arz;
            matA.at<float>(i, 1) = arx;
            matA.at<float>(i, 2) = ary;
            matA.at<float>(i, 3) = coeff.z;
            matA.at<float>(i, 4) = coeff.x;
            matA.at<float>(i, 5) = coeff.y;
            matB.at<float>(i, 0) = -coeff.intensity;
        }

        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0) {

            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate = false;
            float eignThre[6] = {100, 100, 100, 100, 100, 100};
            for (int i = 5; i >= 0; i--) {
                if (matE.at<float>(0, i) < eignThre[i]) {
                    for (int j = 0; j < 6; j++) {
                        matV2.at<float>(i, j) = 0;
                    }
                    isDegenerate = true;
                } else {
                    break;
                }
            }
            matP = matV.inv() * matV2;
        }

        if (isDegenerate)
        {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }

        transformTobeMapped[0] += matX.at<float>(0, 0);
        transformTobeMapped[1] += matX.at<float>(1, 0);
        transformTobeMapped[2] += matX.at<float>(2, 0);
        transformTobeMapped[3] += matX.at<float>(3, 0);
        transformTobeMapped[4] += matX.at<float>(4, 0);
        transformTobeMapped[5] += matX.at<float>(5, 0);

        float deltaR = sqrt(
                            pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
        float deltaT = sqrt(
                            pow(matX.at<float>(3, 0) * 100, 2) +
                            pow(matX.at<float>(4, 0) * 100, 2) +
                            pow(matX.at<float>(5, 0) * 100, 2));

        if (deltaR < 0.05 && deltaT < 0.05) {
            return true; // converged
        }
        return false; // keep optimizing
    }

    void scan2MapOptimization()
    {
        if (cloudKeyPoses3D->points.empty())
            return;

        if (laserCloudCornerLastDSNum > edgeFeatureMinValidNum && laserCloudSurfLastDSNum > surfFeatureMinValidNum)
        {
            kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
            kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

            for (int iterCount = 0; iterCount < 30; iterCount++)
            {
                laserCloudOri->clear();
                coeffSel->clear();

                cornerOptimization();
                surfOptimization();

                combineOptimizationCoeffs();

                if (LMOptimization(iterCount) == true)
                    break;              
            }

            transformUpdate();
        } else {
            ROS_WARN("Not enough features! Only %d edge and %d planar features available.", laserCloudCornerLastDSNum, laserCloudSurfLastDSNum);
        }
    }

    void transformUpdate()
    {
        if (cloudInfo.imuAvailable == true)
        {
            if (std::abs(cloudInfo.imuPitchInit) < 1.4)
            {
                double imuWeight = imuRPYWeight;
                tf::Quaternion imuQuaternion;
                tf::Quaternion transformQuaternion;
                double rollMid, pitchMid, yawMid;

                // slerp roll
                transformQuaternion.setRPY(transformTobeMapped[0], 0, 0);
                imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
                tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[0] = rollMid;

                // slerp pitch
                transformQuaternion.setRPY(0, transformTobeMapped[1], 0);
                imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
                tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[1] = pitchMid;
            }
        }

        transformTobeMapped[0] = constraintTransformation(transformTobeMapped[0], rotation_tollerance);
        transformTobeMapped[1] = constraintTransformation(transformTobeMapped[1], rotation_tollerance);
        transformTobeMapped[5] = constraintTransformation(transformTobeMapped[5], z_tollerance);

        incrementalOdometryAffineBack = trans2Affine3f(transformTobeMapped);
    }

    float constraintTransformation(float value, float limit)
    {
        if (value < -limit)
            value = -limit;
        if (value > limit)
            value = limit;

        return value;
    }

    bool saveFrame()
    {
        if (cloudKeyPoses3D->points.empty())
            return true;

        Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
        Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

        if (abs(roll)  < surroundingkeyframeAddingAngleThreshold &&
            abs(pitch) < surroundingkeyframeAddingAngleThreshold && 
            abs(yaw)   < surroundingkeyframeAddingAngleThreshold &&
            sqrt(x*x + y*y + z*z) < surroundingkeyframeAddingDistThreshold)
            return false;

        return true;
    }

    void addOdomFactor()
    {
        /*当全局第一帧到来时，经过updateInitialGuess()函数的初始化处理后，由于cloudKeyPoses3D为空，其他函数都不处理，然后进入
            该函数，在因子图优化的头，加入先验因子。其值为transformTobeMapped，就是updateInitialGuess()函数处理时的位置，为与
            雷达帧时刻对应的imu时刻的roll、pitch、（yaw或0）、0、0、0
            分析其先验因子的协方差，roll、pitch处的方差较小，表示信任程度较大，yaw处的方差较大，表示信任程度一般，而xyz处方差很大
            表示信任程度很低，因此xyz处的位姿十分依赖于GPS的数据将其拉正。
        */
        if (cloudKeyPoses3D->points.empty())
        {
            std::cout << "add first pose node begin" << std::endl;
            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-4, 1e-4, 1e-4, 1e-6, 1e-6, 1e-6).finished()); // rad*rad, meter*meter
            //noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
            gtSAMgraph.add(PriorFactor<Pose3>(X(0), trans2gtsamPose(transformTobeMapped), priorNoise));
            initialEstimate.insert(X(0), trans2gtsamPose(transformTobeMapped));
            std::cout << "add first pose node finished" << std::endl;
            writeVertex(0, trans2gtsamPose(transformTobeMapped));

        }else{
            noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
            gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
            gtsam::Pose3 poseTo   = trans2gtsamPose(transformTobeMapped);
            gtsam::Pose3 relPose = poseFrom.between(poseTo);
            gtSAMgraph.add(BetweenFactor<Pose3>(X(cloudKeyPoses3D->size()-1), X(cloudKeyPoses3D->size()), relPose, odometryNoise));
            initialEstimate.insert(X(cloudKeyPoses3D->size()), poseTo);

            writeVertex(cloudKeyPoses3D->size(), poseTo);
            writeEdge({cloudKeyPoses3D->size()-1, cloudKeyPoses3D->size()}, relPose); // giseop
        }
    }

    void addGPSFactor()
    {
        if (gpsQueue.empty())
            return;

        // wait for system initialized and settles down
        if (cloudKeyPoses3D->points.empty())
            return;
        else
        {
            if (pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back()) < 5.0)
                return;
        }

        // pose covariance small, no need to correct
        if (poseCovariance(3,3) < poseCovThreshold && poseCovariance(4,4) < poseCovThreshold)
            return;

        // last gps position
        static PointType lastGPSPoint;

        while (!gpsQueue.empty())
        {
            if (gpsQueue.front().header.stamp.toSec() < timeLaserInfoCur - 0.2)
            {
                // message too old
                gpsQueue.pop_front();
            }
            else if (gpsQueue.front().header.stamp.toSec() > timeLaserInfoCur + 0.2)
            {
                // message too new
                break;
            }
            else
            {
                nav_msgs::Odometry thisGPS = gpsQueue.front();
                gpsQueue.pop_front();

                // GPS too noisy, skip
                float noise_x = thisGPS.pose.covariance[0];
                float noise_y = thisGPS.pose.covariance[7];
                float noise_z = thisGPS.pose.covariance[14];
                if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold)
                    continue;

                float gps_x = thisGPS.pose.pose.position.x;
                float gps_y = thisGPS.pose.pose.position.y;
                float gps_z = thisGPS.pose.pose.position.z;
                if (!useGpsElevation)
                {
                    gps_z = transformTobeMapped[5];
                    noise_z = 0.01;
                }

                // GPS not properly initialized (0,0,0)
                if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
                    continue;

                // Add GPS every a few meters
                PointType curGPSPoint;
                curGPSPoint.x = gps_x;
                curGPSPoint.y = gps_y;
                curGPSPoint.z = gps_z;
                if (pointDistance(curGPSPoint, lastGPSPoint) < 5.0)
                    continue;
                else
                    lastGPSPoint = curGPSPoint;

                gtsam::Vector Vector3(3);
                Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
                noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances(Vector3);
                gtsam::GPSFactor gps_factor(X(cloudKeyPoses3D->size()), gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
                gtSAMgraph.add(gps_factor);

                aLoopIsClosed = true;
                break;
            }
        }
    }

    void addPlaneFactor()
    {
        
        //当第一次检测地面点时，planeCurrentQueue队列中有一个平面属性，planeSubmapQueue为空
        if(planeCurrentQueue.empty() || planeSubmapQueue.empty()){
            return;
        }

        if(cloudKeyPoses3D->points.empty()){
            return;
        }
        Eigen::Vector4d planeCurrent;
        Eigen::Vector4d planeSubmap;
        if(planeSubmapQueue.size() != planeCurrentQueue.size()){
            planeCurrentQueue.pop_front();
            planeCurrent = planeCurrentQueue.front();
            planeSubmap = planeSubmapQueue.front();
            planeCurrentQueue.pop_front();
            planeSubmapQueue.pop_front();
            plane_Index++;
        }else{
            planeCurrent = planeCurrentQueue.front();
            planeSubmap = planeSubmapQueue.front();
            planeCurrentQueue.pop_front();
            planeSubmapQueue.pop_front();
            plane_Index++;
        }
        /*
        if(cloudKeyPoses3D->size() < 50){
            return;
        }
        */
       
        OrientedPlane3 Plane3submap(planeSubmap(0), planeSubmap(1), planeSubmap(2), planeSubmap(3));
        //这里对局部地图地面节点加入先验约束的目的是为了fix该节点，优化时只优化位姿节点
        noiseModel::Diagonal::shared_ptr PriorModel = noiseModel::Diagonal::Variances(Vector3(1e-4, 1e-4, 1e-4));
        gtSAMgraph.add(PriorFactor<OrientedPlane3>(P(plane_Index  - 1), Plane3submap, PriorModel));
        initialEstimate.insert(P(plane_Index  - 1), Plane3submap);

        noiseModel::Diagonal::shared_ptr PlaneModel = noiseModel::Diagonal::Variances(Vector3(groundnoiseXY, groundnoiseXY, groundnoiseZ));
        OrientedPlane3Factor plane_factor(planeCurrent, PlaneModel, X(cloudKeyPoses3D->size()), P(plane_Index  - 1));
        gtSAMgraph.add(plane_factor);

    }
    void addLoopFactor()
    {
        if (loopIndexQueue.empty())
            return;

        for (int i = 0; i < (int)loopIndexQueue.size(); ++i)
        {
            int indexFrom = loopIndexQueue[i].first;
            int indexTo = loopIndexQueue[i].second;
            gtsam::Pose3 poseBetween = loopPoseQueue[i];//残差 ：当前帧位姿Twc * 通过icp匹配得到的矫正位姿Tcp - 候选关键帧位姿Twp
            // gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i]; // original 
            auto noiseBetween = loopNoiseQueue[i]; // giseop for polymorhpism // shared_ptr<gtsam::noiseModel::Base>, typedef noiseModel::Base::shared_ptr gtsam::SharedNoiseModel
            gtSAMgraph.add(BetweenFactor<Pose3>(X(indexFrom), X(indexTo), poseBetween, noiseBetween));

            writeEdge({indexFrom, indexTo}, poseBetween); // giseop
        }

        loopIndexQueue.clear();
        loopPoseQueue.clear();
        loopNoiseQueue.clear();

        aLoopIsClosed = true;
    }

    void saveKeyFramesAndFactor()
    {
        
        if (saveFrame() == false)//选择关键帧，若不是关键帧则跳过以下的部分
            return;
        
        // odom factor
        addOdomFactor();

        // gps factor
        addGPSFactor();

        // plane factor
        addPlaneFactor();

        // loop factor
        addLoopFactor(); // radius search loop factor (I changed the orignal func name addLoopFactor to addLoopFactor)

        // update iSAM
        std::cout << "first ISAM2 begin" << std::endl;
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();
        std::cout << "first ISAM2 end" << std::endl;

        if (aLoopIsClosed == true)
        {
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            isam->update();
        }

        gtSAMgraph.resize(0);
        initialEstimate.clear();

        //save key poses
        PointXYZIRT thisPose3D;
        PointTypePose thisPose6D;
        Pose3 latestEstimate;
        
        std::cout << "isam->calculateEstimate() begin" << std::endl;
        isamCurrentEstimate = isam->calculateEstimate();
        std::cout << "isam->calculateEstimate() end" << std::endl;
        //新添加关键帧的时间戳
        std::ofstream timeStream (savePCDDirectory + "keyframeTime.txt",  std::ios::app);
        timeStream.setf(std::ios::scientific, std::ios::floatfield);
        timeStream.precision(12);
        timeStream << timeLaserInfoCur <<std::endl;

        std::ofstream timeSIMStream (savePCDDirectory + "keyframeSIMTime.txt",  std::ios::app);
        timeSIMStream.setf(std::ios::scientific, std::ios::floatfield);
        timeSIMStream.precision(12);
        timeSIMStream << simTime <<std::endl;

        //**********************************************************
        std::cout << "select data begin" << std::endl;
        std::cout << "isamCurrentEstimate.size(): " << isamCurrentEstimate.size()  <<std::endl;
        std::cout << "plane_Index: " << plane_Index  <<std::endl;
        latestEstimate = isamCurrentEstimate.at<Pose3>(X(isamCurrentEstimate.size()- plane_Index -1));
        std::cout << "select data end" << std::endl;

        // cout << "****************************************************" << endl;
        // isamCurrentEstimate.print("Current estimate: ");

        thisPose3D.x = latestEstimate.translation().x();
        thisPose3D.y = latestEstimate.translation().y();
        thisPose3D.z = latestEstimate.translation().z();
        //这个intensity不是包含的点云的数量，而是数量减1，因为push_back在赋值后面 也就是索引
        thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as index 
        if(flagForPlaneExtract){
            std::cout << "dectectPlaneCounts: " << dectectPlaneCounts << std::endl;
            thisPose3D.ring = cloudKeyPoses3D->size() + dectectPlaneCounts;//有地面点检测的帧在全局的索引 + (有地面点检测的帧在地面帧群中的索引 + 1)(有地面点检测帧的数量) 
            std::cout << "thisPose3D.intensity: " << thisPose3D.intensity << std::endl;
            std::cout << "thisPose3D.ring: " << thisPose3D.ring << std::endl;
        }else{
            thisPose3D.ring = 0;
        }
        cloudKeyPoses3D->push_back(thisPose3D);
        
        flagForPlaneExtract = false;

        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity ; // this can be used as index
        thisPose6D.roll  = latestEstimate.rotation().roll();
        thisPose6D.pitch = latestEstimate.rotation().pitch();
        thisPose6D.yaw   = latestEstimate.rotation().yaw();
        thisPose6D.time = timeLaserInfoCur;
        cloudKeyPoses6D->push_back(thisPose6D);

        // cout << "****************************************************" << endl;
        // cout << "Pose covariance:" << endl;
        // cout << isam->marginalCovariance(isamCurrentEstimate.size()-1) << endl << endl;
        //std::cout << "marginalCovariance begin: " << isamCurrentEstimate.size()  <<std::endl;
        poseCovariance = isam->marginalCovariance(X(isamCurrentEstimate.size() - plane_Index -1));
        //std::cout << "marginalCovariance end: " << isamCurrentEstimate.size()  <<std::endl;

        // save updated transform
        transformTobeMapped[0] = latestEstimate.rotation().roll();
        transformTobeMapped[1] = latestEstimate.rotation().pitch();
        transformTobeMapped[2] = latestEstimate.rotation().yaw();
        transformTobeMapped[3] = latestEstimate.translation().x();
        transformTobeMapped[4] = latestEstimate.translation().y();
        transformTobeMapped[5] = latestEstimate.translation().z();


        // save all the received edge and surf points
        pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*laserCloudCornerLastDS,  *thisCornerKeyFrame);
        pcl::copyPointCloud(*laserCloudSurfLastDS,    *thisSurfKeyFrame);

        // save key frame cloud
        cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
        surfCloudKeyFrames.push_back(thisSurfKeyFrame);

        // Scan Context loop detector - giseop
        // - SINGLE_SCAN_FULL: using downsampled original point cloud (/full_cloud_projected + downsampling)
        // - SINGLE_SCAN_FEAT: using surface feature as an input point cloud for scan context (2020.04.01: checked it works.)
        // - MULTI_SCAN_FEAT: using NearKeyframes (because a MulRan scan does not have beyond region, so to solve this issue ... )
        const SCInputType sc_input_type = SCInputType::SINGLE_SCAN_FULL; // change this 

        if( sc_input_type == SCInputType::SINGLE_SCAN_FULL ) {
            pcl::PointCloud<PointType>::Ptr thisRawCloudKeyFrame(new pcl::PointCloud<PointType>());
            pcl::copyPointCloud(*laserCloudRawDS,  *thisRawCloudKeyFrame);//当前帧降采样后的所有点
            scManager.makeAndSaveScancontextAndKeys(*thisRawCloudKeyFrame);
        }  
        else if (sc_input_type == SCInputType::SINGLE_SCAN_FEAT) { 
            scManager.makeAndSaveScancontextAndKeys(*thisSurfKeyFrame); 
        }
        else if (sc_input_type == SCInputType::MULTI_SCAN_FEAT) { 
            pcl::PointCloud<PointType>::Ptr multiKeyFrameFeatureCloud(new pcl::PointCloud<PointType>());
            loopFindNearKeyframes(multiKeyFrameFeatureCloud, cloudKeyPoses6D->size() - 1, historyKeyframeSearchNum);
            scManager.makeAndSaveScancontextAndKeys(*multiKeyFrameFeatureCloud); 
        }

        // save sc data
        const auto& curr_scd = scManager.getConstRefRecentSCD();
        std::string curr_scd_node_idx = padZeros(scManager.polarcontexts_.size() - 1);

        saveSCD(saveSCDDirectory + curr_scd_node_idx + ".scd", curr_scd);


        // save keyframe cloud as file giseop
        bool saveRawCloud { true };
        pcl::PointCloud<PointType>::Ptr thisKeyFrameCloud(new pcl::PointCloud<PointType>());
        if(saveRawCloud) { 
            *thisKeyFrameCloud += *laserCloudRaw;
        } else {
            *thisKeyFrameCloud += *thisCornerKeyFrame;
            *thisKeyFrameCloud += *thisSurfKeyFrame;
        }
        pcl::io::savePCDFileBinary(saveCornerPCDDirectory + curr_scd_node_idx + ".pcd", *thisCornerKeyFrame);
        pcl::io::savePCDFileBinary(saveSurfPCDDirectory + curr_scd_node_idx + ".pcd", *thisSurfKeyFrame);
        pcl::io::savePCDFileBinary(saveNodePCDDirectory + curr_scd_node_idx + ".pcd", *thisKeyFrameCloud);
        pgTimeSaveStream << laserCloudRawTime << std::endl;

        // save path for visualization
        updatePath(thisPose6D);
        //std::cout << "updatePath finished" << std::endl;
    }

    void correctPoses()
    {
        //std::cout << "correctPoses begin" << std::endl;
        if (cloudKeyPoses3D->points.empty())
            return;

        if (aLoopIsClosed == true)
        {
            // clear map cache
            laserCloudMapContainer.clear();
            laserCloudPlaneMapContainer.clear();
            // clear path
            globalPath.poses.clear();
            // update key poses
            int numPoses = isamCurrentEstimate.size() - plane_Index;
            for (int i = 0; i < numPoses; ++i)
            {
                cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(X(i)).translation().x();
                cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(X(i)).translation().y();
                cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(X(i)).translation().z();

                cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
                cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
                cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
                cloudKeyPoses6D->points[i].roll  = isamCurrentEstimate.at<Pose3>(X(i)).rotation().roll();
                cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(X(i)).rotation().pitch();
                cloudKeyPoses6D->points[i].yaw   = isamCurrentEstimate.at<Pose3>(X(i)).rotation().yaw();

                updatePath(cloudKeyPoses6D->points[i]);
            }

            aLoopIsClosed = false;
        }
    }

    void updatePath(const PointTypePose& pose_in)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time().fromSec(pose_in.time);
        pose_stamped.header.frame_id = odometryFrame;
        pose_stamped.pose.position.x = pose_in.x;
        pose_stamped.pose.position.y = pose_in.y;
        pose_stamped.pose.position.z = pose_in.z;
        tf::Quaternion q = tf::createQuaternionFromRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        globalPath.poses.push_back(pose_stamped);
    }

    void publishOdometry()
    {
        //std::cout << "publishOdometry begin" << std::endl;
        // Publish odometry for ROS (global)
        nav_msgs::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp = timeLaserInfoStamp;
        laserOdometryROS.header.frame_id = odometryFrame;
        laserOdometryROS.child_frame_id = "odom_mapping";
        laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
        laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
        laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
        laserOdometryROS.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        pubLaserOdometryGlobal.publish(laserOdometryROS);
        
        // Publish TF
        static tf::TransformBroadcaster br;
        tf::Transform t_odom_to_lidar = tf::Transform(tf::createQuaternionFromRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]),
                                                      tf::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
        tf::StampedTransform trans_odom_to_lidar = tf::StampedTransform(t_odom_to_lidar, timeLaserInfoStamp, odometryFrame, "lidar_link");
        br.sendTransform(trans_odom_to_lidar);

        // Publish odometry for ROS (incremental)
        static bool lastIncreOdomPubFlag = false;
        static nav_msgs::Odometry laserOdomIncremental; // incremental odometry msg
        static Eigen::Affine3f increOdomAffine; // incremental odometry in affine
        if (lastIncreOdomPubFlag == false)
        {
            lastIncreOdomPubFlag = true;
            laserOdomIncremental = laserOdometryROS;
            increOdomAffine = trans2Affine3f(transformTobeMapped);
        } else {
            Eigen::Affine3f affineIncre = incrementalOdometryAffineFront.inverse() * incrementalOdometryAffineBack;
            increOdomAffine = increOdomAffine * affineIncre;
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles (increOdomAffine, x, y, z, roll, pitch, yaw);
            if (cloudInfo.imuAvailable == true)
            {
                if (std::abs(cloudInfo.imuPitchInit) < 1.4)
                {
                    double imuWeight = 0.1;
                    tf::Quaternion imuQuaternion;
                    tf::Quaternion transformQuaternion;
                    double rollMid, pitchMid, yawMid;

                    // slerp roll
                    transformQuaternion.setRPY(roll, 0, 0);
                    imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
                    tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    roll = rollMid;

                    // slerp pitch
                    transformQuaternion.setRPY(0, pitch, 0);
                    imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
                    tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    pitch = pitchMid;
                }
            }
            laserOdomIncremental.header.stamp = timeLaserInfoStamp;
            laserOdomIncremental.header.frame_id = odometryFrame;
            laserOdomIncremental.child_frame_id = "odom_mapping";
            laserOdomIncremental.pose.pose.position.x = x;
            laserOdomIncremental.pose.pose.position.y = y;
            laserOdomIncremental.pose.pose.position.z = z;
            laserOdomIncremental.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
            if (isDegenerate)
                laserOdomIncremental.pose.covariance[0] = 1;
            else
                laserOdomIncremental.pose.covariance[0] = 0;
        }
        std::ofstream output3("/home/gky/桌面/slam_results/liosamsc_lidarPose.csv",  std::ios::app);

        output3.setf(std::ios::scientific, std::ios::floatfield);
        output3.precision(6);
        output3 << timeLaserInfoStamp << "," << laserOdomIncremental.pose.pose.position.x << "," << laserOdomIncremental.pose.pose.position.y << "," << laserOdomIncremental.pose.pose.position.z << ","
                       <<  laserOdomIncremental.pose.pose.orientation.x << "," << laserOdomIncremental.pose.pose.orientation.y << "," << laserOdomIncremental.pose.pose.orientation.z << "," << laserOdomIncremental.pose.pose.orientation.w << endl;
        pubLaserOdometryIncremental.publish(laserOdomIncremental);
    }

    void publishFrames()
    {
        if (cloudKeyPoses3D->points.empty())
            return;
        //publish submap plane points
        publishCloud(&pubRecentPlanePoints, laserCloudPlaneFromMapDS, timeLaserInfoStamp, odometryFrame);
        // publish key poses
        publishCloud(&pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp, odometryFrame);
        // Publish surrounding key frames
        publishCloud(&pubRecentKeyFrames, laserCloudSurfFromMapDS, timeLaserInfoStamp, odometryFrame);
        ROS_INFO_STREAM("上一帧的局部地图点云数量为: \033[1;34m" << laserCloudSurfFromMapDS->size() << "\033[0m");

        // publish registered key frame
        if (pubRecentKeyFrame.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut += *transformPointCloud(laserCloudCornerLastDS,  &thisPose6D);
            *cloudOut += *transformPointCloud(laserCloudSurfLastDS,    &thisPose6D);
            publishCloud(&pubRecentKeyFrame, cloudOut, timeLaserInfoStamp, odometryFrame);
        }
        // publish registered high-res raw cloud
        if (pubCloudRegisteredRaw.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloudOut);
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut = *transformPointCloud(cloudOut,  &thisPose6D);
            publishCloud(&pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp, odometryFrame);
        }
        // publish path
        if (pubPath.getNumSubscribers() != 0)
        {
            globalPath.header.stamp = timeLaserInfoStamp;
            globalPath.header.frame_id = odometryFrame;
            pubPath.publish(globalPath);
        }
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_sam");
    setlocale(LC_CTYPE, "zh_CN.utf8");
    mapOptimization MO;

    ROS_INFO("\033[1;32m----> Map Optimization Started.\033[0m");
    
    std::thread loopthread(&mapOptimization::loopClosureThread, &MO);
    std::thread visualizeMapThread(&mapOptimization::visualizeGlobalMapThread, &MO);

    ros::spin();

    loopthread.join();
    visualizeMapThread.join();

    return 0;
}
