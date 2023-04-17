/*
*******************************************************************************
* communication_center.cpp:
* This code is used to communicate between Observer and DP Main.
*   Observer:   Qualisys
*
* Update By Hu.CJ
* Date 2023.02.12
*******************************************************************************
*/
// include for Qualisys and KMDP communicate
#include <unistd.h>
#include <math.h>
#include <map>
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/TransformStamped.h>
// #include <nav_msgs/Odometry.h>
#include "RTProtocol.h"
#include "RTPacket.h"
#include <stdio.h>
#include <time.h>
#include <ctime>
#include <sys/timeb.h>
#include "enping_ship/msg_ship_pose.h"
//
/* Main function */
int main(int argc, char **argv)
{
    using namespace std;

    // Set up ROS.
    ros::init(argc, argv, "qualisys_pose");
    ros::NodeHandle nh("~");

    // declare variables that can be modified by launch file or command line.
    string server;
    double rate_limit;
    // initialize parameters from launch file or command line.
    nh.param("server", server, string("172.21.101.201")); //"172.21.101.70")
    nh.param("rate_limit", rate_limit, 10.0);

    /* sub and pub */
    ros::Publisher pub_qualisys_pose = nh.advertise<enping_ship::msg_ship_pose>("qualisys_pose", 1);
    enping_ship::msg_ship_pose msg_qualisys_pose;

    // connect qualisys
    try
    {
        CRTProtocol rtProtocol;

        const unsigned short basePort = 22222;
        const int majorVersion = 1;
        const int minorVersion = 13;
        const bool bigEndian = false;
        bool dataAvailable = false;
        bool streamFrames = false;
        // unsigned short udpPort = 6734;
        unsigned short udpPort = 6734;

        ros::Rate rate_20HZ(50);

        // Main loop.
        while (nh.ok())
        {
            if (!rtProtocol.Connected())
            {
                std::cout << "hhhhhhhhhhhh" << std::endl;
                if (!rtProtocol.Connect(server.c_str(), basePort, &udpPort, majorVersion, minorVersion, bigEndian))
                {
                    ROS_WARN("rtProtocol.Connect: %s\n\n", rtProtocol.GetErrorString());
                    sleep(1);
                    continue;
                }
            }

            if (!dataAvailable)
            {
                if (!rtProtocol.Read6DOFSettings(dataAvailable))
                {
                    ROS_WARN("rtProtocol.Read6DOFSettings: %s\n\n", rtProtocol.GetErrorString());
                    sleep(1);
                    continue;
                }
            }

            if (!streamFrames)
            {
                if (!rtProtocol.StreamFrames(CRTProtocol::RateAllFrames, 0, udpPort, NULL, CRTProtocol::cComponent6d))
                {
                    ROS_WARN("rtProtocol.StreamFrames: %s\n\n", rtProtocol.GetErrorString());
                    sleep(1);
                    continue;
                }
                streamFrames = true;

                ROS_INFO("Starting to streaming 6DOF data");
            }

            CRTPacket::EPacketType packetType;

            if (rtProtocol.ReceiveRTPacket(packetType, true) > 0)
            {
                if (packetType == CRTPacket::PacketData)
                {
                    float fX, fY, fZ;
                    float rotationMatrix[9];

                    CRTPacket *rtPacket = rtProtocol.GetRTPacket();

                    // ROS_WARN("Frame %d\n", rtPacket->GetFrameNumber());
                    for (unsigned int i = 0; i < rtPacket->Get6DOFBodyCount(); i++)
                    {

                        if (rtPacket->Get6DOFBody(i, fX, fY, fZ, rotationMatrix))
                        {
                            string name(rtProtocol.Get6DOFBodyName(i));
                            // ROS_WARN("data received for rigid body %s", name.c_str());
                            std::cout << "================name=============" << name << "================================" << std::endl;
                            if (name != "Barge") // Barge
                            {
                                continue;
                            };
                            std::cout << "fxfyfz," << fX << "," << fY << "," << fZ << std::endl;
                            if (!isfinite(fX) || !isfinite(fY) || !isfinite(fZ))
                            {
                                ROS_WARN_THROTTLE(3, "rigid body %s tracking lost", name.c_str());
                                continue;
                            }

                            for (int i = 0; i < 9; i++)
                            {
                                std::cout << "bodycount" << rtPacket->Get6DOFBodyCount() << std::endl;

                                if (!isfinite(rotationMatrix[i]))
                                {
                                    ROS_WARN_THROTTLE(3, "rigid body %s tracking lost", name.c_str());
                                    continue;
                                }
                            }

                            ros::Time now = ros::Time::now();

                            // convert to quaternion
                            tf2::Matrix3x3 R(
                                rotationMatrix[0], rotationMatrix[3], rotationMatrix[6],
                                rotationMatrix[1], rotationMatrix[4], rotationMatrix[7],
                                rotationMatrix[2], rotationMatrix[5], rotationMatrix[8]);
                            // std::cout << "矩阵" << std::endl;
                            // std::cout << rotationMatrix[0] << "," << rotationMatrix[1] << "," << rotationMatrix[2] << std::endl;
                            // std::cout << rotationMatrix[3] << "," << rotationMatrix[4] << "," << rotationMatrix[5] << std::endl;
                            // std::cout << rotationMatrix[6] << "," << rotationMatrix[7] << "," << rotationMatrix[8] << std::endl;

                            tf2::Quaternion q;
                            R.getRotation(q);

                            double roll, pitch, yaw;
                            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
                            std::cout << " origin roll,pitch,yaw-----" << roll << ",  " << pitch << ",  " << yaw << std::endl;
                            std::cout << "origin x,y,z:-------" << fX << ",  " << fY << ",  " << fZ << std::endl;
                            double pi = 3.1415926;

                            while (yaw > pi || yaw < -pi)
                            {
                                if (yaw > pi)
                                    yaw -= 2 * pi;
                                if (yaw < -pi)
                                    yaw += 2 * pi;
                            };

                            // scale position to meters from mm
                            double x = (fX / 1.0e3) * 36;
                            double y = (fY / 1.0e3) * (36); //[TBD]
                            double z = fZ / 1.0e3;
                            double elapsed = 0;
                            std::cout << "******************Qualisys************************\n";
                            std::cout << "roll,pitch,yaw-----" << roll << ",  " << pitch << ",  " << yaw << std::endl;
                            std::cout << "x,y,z:-------" << x << ",  " << y << ",  " << z << std::endl;

                            msg_qualisys_pose.x = x;
                            msg_qualisys_pose.y = y;
                            msg_qualisys_pose.psi = yaw;
                            pub_qualisys_pose.publish(msg_qualisys_pose);
                        }
                    }
                }
            } // end if ReceiveRTPacket

            rate_20HZ.sleep();
        }
        ros::spinOnce();
        rtProtocol.StreamFramesStop();
        rtProtocol.Disconnect();
    }
    catch (std::exception &e)
    {
        printf("%s\n", e.what());
    }
    return 1;

    return 0;
} // end main;