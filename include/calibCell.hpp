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

/*!
 * \brief The CalibrateCell class
 */
class CalibrateCell{
public:
    /*!
     * \brief Constructor for CalibrateCell class.
     * \param argc Initialization argument
     * \param argv Initialization argument
     */
    CalibrateCell(int argc, char** argv);

    /*!
     * \brief Deconstructor for CalibrateCell class.
     */
    virtual ~CalibrateCell();

    /*!
     * \brief Initialization method for the ROS node.
     * \return Returns true if initialization is OK. Returns false if
     * initialization fails.
     */
    bool init();

    /*!
     * \brief Starts the ROS node and loops.
     */
    void run();

    /*!
     * \brief Callback method from ar_track_alvar
     * \param cam_pos position of the AR tag relative to the camera
     */
    void camPosCallback1(const ar_track_alvar_msgs::AlvarMarkersConstPtr& cam_pos);

private:
    int init_argc; //!< Initialization arguments
    char** init_argv; //!< Initialization arguments
    ros::Subscriber subPose; //!< ROS subscriber to the pose message
    int nrOfMsgs; //!< Number of messages to average
    int msgs; //!< Current messages averaged
    std::vector<tf::Vector3> positions; //!< Vector containing the pose position (X,Y,Z)
    std::vector<tf::Quaternion> quaternions; //!< Vector containing the pose orientation in quaternions
    std::string cameraName; //!< Name of the camera
    std::vector<Eigen::Matrix4f> transformationMatrices; //!< Vector containing the pose in transformation matrices

    /*!
     * \brief Converts a position and quaternion vector to a 4x4 transformation matrix.
     * \param cameraID the name of the camera
     * \param position the pose position vector
     * \param quaternion the pose orientation vector in quaternions
     * \return
     */
    Eigen::Matrix4f calcTransformationMatrix(std::string cameraID, tf::Vector3 position, tf::Quaternion quaternion);

    /*!
     * \brief Adds a new transformation (position and orientation) to the private vectors positions and quaternions.
     * \param cameraID the name of the camera
     * \param position the pose position vector
     * \param quaternion the pose orientation vector in quaternions
     */
    void addNewTransformation(std::string cameraID, tf::Vector3 position, tf::Quaternion quaternion);

    /*!
     * \brief Takes the average of the vector of position and orientation and writes the
     * result as a 4x4 transformation matrix to a .txt file.
     */
    void averageRotations();
};
