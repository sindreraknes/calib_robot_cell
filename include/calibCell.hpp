#include <ros/ros.h>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <fstream>


class CalibrateCell{
public:
    CalibrateCell(int argc, char** argv);
    virtual ~CalibrateCell();
    bool init();
    void run();

    void camPosCallback1(const ar_track_alvar_msgs::AlvarMarkersConstPtr& cam_pos);


private:
    ros::Subscriber subPose;

    int nrOfMsgs;
    int msgs;

    std::vector<tf::Vector3> positions;
    std::vector<tf::Quaternion> quaternions;
    std::string cameraName;
    std::vector<Eigen::Matrix4f> transformationMatrices;

    Eigen::Matrix4f calcTransformationMatrix(std::string cameraID, tf::Vector3 position, tf::Quaternion quaternion);

    void addNewTransformation(std::string cameraID, tf::Vector3 position, tf::Quaternion quaternion);

    void averageRotations();

    int init_argc;
    char** init_argv;

};
