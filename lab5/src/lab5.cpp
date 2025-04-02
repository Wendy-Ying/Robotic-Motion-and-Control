#include <ros/ros.h>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>

using namespace Eigen;

Matrix4f dhTransform(float alpha, float a, float d, float theta) {
    Matrix4f T;
    T << cos(theta), -sin(theta), 0, a,
         sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -d * sin(alpha),
         sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha), d * cos(alpha),
         0, 0, 0, 1;
    return T;
}

void getDHTable(float dhtable[6][4]) {
    const double pi = std::acos(-1);
    float temp[6][4] = {
        {0.0f, 0.0f, 243.3f, 0.0f},
        {pi / 2, 0.0f, 30.0f, pi / 2},
        {pi, 280.0f, 20.0f, pi / 2},
        {pi / 2, 0.0f, 245.0f, pi / 2},
        {pi / 2, 0.0f, 57.0f, 0.0f},
        {-pi / 2, 0.0f, 235.0f, -pi / 2}
    };
    std::copy(&temp[0][0], &temp[0][0] + 6 * 4, &dhtable[0][0]);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lab5");
    ros::NodeHandle nh;

    if (argc != 7) {
        ROS_ERROR("Usage: theta1 theta2 theta3 theta4 theta5 theta6");
        return -1;
    }

    const double pi = std::acos(-1);
    float theta[6];
    for (int i = 0; i < 6; ++i) {
        theta[i] = std::stof(argv[i + 1]) * pi / 180.0f; // change from degree to radian
    }

    float dhtable[6][4];
    getDHTable(dhtable);

    // print D-H table
    std::ostringstream table;
    ROS_INFO("D-H Table...");
    table << std::setw(5) << "Linkage"
          << std::setw(15) << "alpha i-1" 
          << std::setw(15) << "a i-1"
          << std::setw(15) << "d i" 
          << std::setw(20) << "theta i" 
          << "\n";

    table << std::string(75, '-') << "\n";

    for (int i = 1; i <= 6; i++) {
        table << std::setw(5) << "link" << i
              << std::setw(15) << dhtable[i-1][0]
              << std::setw(15) << dhtable[i-1][1]
              << std::setw(15) << dhtable[i-1][2]
              << std::setw(15) << dhtable[i-1][3] << " + theta " << i
              << "\n";
    }

    ROS_INFO_STREAM("\n" << table.str());

    Matrix4f T = Matrix4f::Identity();
    for (int i = 0; i < 6; i++) {
        T *= dhTransform(dhtable[i][0], dhtable[i][1], dhtable[i][2], dhtable[i][3] + theta[i]);
    }

    ROS_INFO("Transformation Matrix (Base to Tip):");
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            std::cout << std::setw(12) << T(i, j) << " ";
        }
        std::cout << std::endl;
    }

    // get end effector position and orientation
    float x = T(0, 3);
    float y = T(1, 3);
    float z = T(2, 3);
    float beta = atan2(-T(2, 0), sqrt(T(0, 0) * T(0, 0) + T(1, 0) * T(1, 0)));
    float alpha = atan2(T(1, 0), T(0, 0));
    float gamma = atan2(T(2, 1), T(2, 2));
    ROS_INFO("End Effector Position: x = %.3f, y = %.3f, z = %.3f", x, y, z);
    ROS_INFO("End Effector Orientation: alpha = %.3f, beta = %.3f, gamma = %.3f", alpha * 180.0 / M_PI, beta * 180.0 / M_PI, gamma * 180.0 / M_PI);

    ros::Publisher ee_pose_pub = nh.advertise<geometry_msgs::Pose>("ee_pose", 10);
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        geometry_msgs::Pose ee_pose;
        ee_pose.position.x = x;
        ee_pose.position.y = y;
        ee_pose.position.z = z;
        ee_pose.orientation.x = alpha * 180.0 / M_PI;
        ee_pose.orientation.y = beta * 180.0 / M_PI;
        ee_pose.orientation.z = gamma * 180.0 / M_PI;
        ee_pose.orientation.w = 0;
        ee_pose_pub.publish(ee_pose);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
