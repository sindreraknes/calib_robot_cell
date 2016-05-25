#include "../include/calibCell.hpp"
#include <iostream>



CalibrateCell::CalibrateCell(int argc, char **argv):
    init_argc(argc),
    init_argv(argv)
{}

CalibrateCell::~CalibrateCell()
{
    if(ros::isStarted()) {
      ros::shutdown();
      ros::waitForShutdown();
    }
}

bool CalibrateCell::init()
{
    std::cout << "Initing" << std::endl;
    nrOfMsgs = 100;
    msgs=0;
    tf::Quaternion q(0, 0, 0, 0);
    tf::Vector3 v(0,0,0);
    positions.push_back(v);
    quaternions.push_back(q);
    cameraName = "";

    ros::init(init_argc,init_argv,"calib_cell");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start();

    ros::NodeHandle n;
    subPose = n.subscribe<ar_track_alvar_msgs::AlvarMarkers, CalibrateCell>("/ar_pose_marker", 10, &CalibrateCell::camPosCallback1, this);

    return true;
}

void CalibrateCell::run()
{
    std::cout << "Running" << std::endl;
    ros::Rate loop_rate(200);
    while ( ros::ok() ){
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void CalibrateCell::camPosCallback1(const ar_track_alvar_msgs::AlvarMarkersConstPtr &cam_pos)
{
    if(cam_pos->markers.size() == 0){
        return;
    }
    // Camera ID
    std::string cameraID = cam_pos->markers.at(0).header.frame_id;
    // Position XYZ
    geometry_msgs::Point positionMsg = cam_pos->markers.at(0).pose.pose.position;
    tf::Vector3 position(positionMsg.x, positionMsg.y, positionMsg.z);
    // Rotation
    geometry_msgs::Quaternion quatMsg = cam_pos->markers.at(0).pose.pose.orientation;
    tf::Quaternion q(quatMsg.x, quatMsg.y, quatMsg.z, quatMsg.w);


    if(msgs == nrOfMsgs){
        std::cout << "Im done" << std::endl;
        subPose.shutdown();
        averageRotations();
        ros::shutdown();

    }
    addNewTransformation(cameraID,position,q);

}

Eigen::Matrix4f CalibrateCell::calcTransformationMatrix(std::string cameraID, tf::Vector3 position, tf::Quaternion quaternion)
{
    tf::Matrix3x3 rotation(quaternion);
    Eigen::Matrix4f transTmp;
    transTmp <<     rotation.getColumn(0).getX(), rotation.getColumn(1).getX(), rotation.getColumn(2).getX(),  position.getX(),
                    rotation.getColumn(0).getY(), rotation.getColumn(1).getY(), rotation.getColumn(2).getY(),  position.getY(),
                    rotation.getColumn(0).getZ(), rotation.getColumn(1).getZ(), rotation.getColumn(2).getZ(),  position.getZ(),
                    0,          0,          0,       1;

    //std::cout << cameraID << std::endl;
    //std::cout << transTmp << std::endl;
    double roll, pitch, yaw;
    std::cout << "Rotation RPY: " << std::endl;
    rotation.getRPY(roll, pitch, yaw);
    std::cout << roll << std::endl;
    std::cout << pitch << std::endl;
    std::cout << yaw << std::endl;

    return transTmp;
}

void CalibrateCell::addNewTransformation(std::string cameraID, tf::Vector3 position, tf::Quaternion quaternion)
{
    if( msgs < nrOfMsgs){
        std::cout << "Added from: ";
        std::cout << cameraID << std::endl;
        std::cout << "Number: ";
        std::cout << msgs << std::endl;
        positions[0] += position;
        quaternions[0] += quaternion;
        msgs++;
        //if(msgs == 1){
            calcTransformationMatrix(cameraID, position, quaternion);
            cameraName = cameraID;
        //}
    }

}

void CalibrateCell::averageRotations()
{
    std::vector<float> x(quaternions.size());
    std::vector<float> y(quaternions.size());
    std::vector<float> z(quaternions.size());
    std::vector<float> w(quaternions.size());

    std::vector<float> xPos(quaternions.size());
    std::vector<float> yPos(quaternions.size());
    std::vector<float> zPos(quaternions.size());

    float div = 1.0f/(float)nrOfMsgs;
    //std::cout << div << std::endl;

    for(int i=0; i<quaternions.size(); i++){
        x[i] = quaternions.at(i).getX()*div;
        y[i] = quaternions.at(i).getY()*div;
        z[i] = quaternions.at(i).getZ()*div;
        w[i] = quaternions.at(i).getW()*div;

        xPos[i] = positions.at(i).getX()*div;
        yPos[i] = positions.at(i).getY()*div;
        zPos[i] = positions.at(i).getZ()*div;
    }

    for(int k=0; k<quaternions.size(); k++){
        tf::Quaternion quat(x.at(k), y.at(k), z.at(k), w.at(k));
        quat.normalize();
        tf::Vector3 position(xPos.at(k), yPos.at(k), zPos.at(k));
        Eigen::Matrix4f tmp = calcTransformationMatrix(cameraName,position, quat);
        transformationMatrices.push_back(tmp);
    }

    char chars[] = "/";

    for (unsigned int i = 0; i < strlen(chars); ++i)
    {
        cameraName.erase (std::remove(cameraName.begin(), cameraName.end(), chars[i]), cameraName.end());
    }

    std::string tmp = "/home/minions/";
    tmp.append(cameraName);
    tmp.append(".txt");

    std::ofstream file(tmp.c_str());
    if (file.is_open())
    {
        for(int i=0; i<transformationMatrices.size(); i++){
            file << "Matrix for: " << cameraName << '\n';
            file << transformationMatrices.at(i) << '\n' << '\n';
        }

    }

    std::cout << "Done calibrating, wrote matrix to file: ";
    std::cout << tmp << std::endl;

}



int main(int argc, char **argv){
    CalibrateCell cell(argc, argv);
    cell.init();
    cell.run();
}




