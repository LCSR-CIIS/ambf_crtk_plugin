//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019-2024

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of authors nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    <hishida3@jhu.edu>
    \author    Hisashi Ishida
    \date      01.10.2024
    
*/
//==============================================================================


#ifndef AFCRTK_INTERFACE_H
#define AFCRTK_INTERFACE_H


#include <math/CTransform.h>

#include <afFramework.h>

// to define AMBF_ROS1, AMBF_ROS2 , AMBF_ROS_DISTRO
#include <ambf_server/ambf_ral_config.h>

#if AMBF_ROS1
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <crtk_msgs/operating_state.h>
#elif AMBF_ROS2
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "crtk_msgs/msg/operating_state.hpp"
#include <ambf_server/ambf_ral.h>
#endif

#include <ambf_server/RosComBase.h>


using namespace chai3d;
using namespace std;

class afCRTKInterface{
public:
    afCRTKInterface(string a_namespace);
    ~afCRTKInterface();

    void init(string a_namespace);
    void add_allInterface(string a_namespace);
    void add_operating_state(string a_namespace);
    void add_measured_cp(string a_namespace);
    void add_setpoint_cp(string a_namespace);
    void add_measured_js(string a_namespace, vector<string> jointName);
    void add_measured_cf(string a_namespace);
    void add_servo_cp(string a_namespace);
    void add_servo_jp(string a_namespace);
    void add_servo_cf(string a_namespace);

    // ROS related
    // ros::NodeHandle* m_rosNode;
    ambf_ral::node_ptr_t m_rosNode;

    // Callback functions
    void servo_CPCallback(AMBF_RAL_MSG_PTR(geometry_msgs, PoseStamped) msg); 
    void servo_JPCallback(AMBF_RAL_MSG_PTR(sensor_msgs, JointState) msg);
    void servo_CFCallback(AMBF_RAL_MSG_PTR(geometry_msgs, WrenchStamped) msg);

    // Query Command
    void run_operating_state();
    void measured_cp(cTransform &trans, string name = "default");
    void setpoint_cp(cTransform &trans, string name = "default");
    void measured_js(vector<double>& q);
    void measured_cf(vector<double>& force, string name = "default");

    // Motion Command
    bool servo_cp(cTransform & cp);
    bool servo_jp(vector<double> & jp);
    bool servo_cf(vector<double> & cf);


private:

    #if AMBF_ROS1
        // Subscribers
        ros::Subscriber m_servoCPSub;
        ros::Subscriber m_servoJPSub;
        ros::Subscriber m_servoCFSub;

        map<string, ros::Subscriber> m_servoCPSubMap;
        map<string, ros::Subscriber> m_servoCFSubMap;

        // Publishers
        ros::Publisher m_operatingStatePub;
        ros::Publisher m_measuredCPPub;
        ros::Publisher m_measuredJSPub;
        ros::Publisher m_measuredCFPub;

        map<string, ros::Publisher> m_measuredCPPubMap;
        map<string, ros::Publisher> m_measuredCFPubMap;

        crtk_msgs::OperatingState m_operatingState;
        geometry_msgs::PoseStamped m_measured_cp;   
        sensor_msgs::JointState m_measured_js;
        geometry_msgs::WrenchStamped m_measured_cf;

    #elif AMBF_ROS2
        // Subscribers
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_servoCPSub; 
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_servoJPSub;
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr m_servoCFSub;

        map<string, rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> m_servoCPSubMap;
        map<string, rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr> m_servoCFSubMap;

        rclcpp::Publisher<crtk_msgs::msg::OperatingState>::SharedPtr m_operatingStatePub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_measuredCPPub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_setpointCPPub;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_measuredJSPub;
        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr m_measuredCFPub;

        map<string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> m_measuredCPPubMap;
        map<string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> m_setpointCPPubMap;
        map<string, rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr> m_measuredCFPubMap;

        crtk_msgs::msg::OperatingState m_operatingState;
        geometry_msgs::msg::PoseStamped m_measured_cp;   
        geometry_msgs::msg::PoseStamped m_setpoint_cp;   
        sensor_msgs::msg::JointState m_measured_js;
        geometry_msgs::msg::WrenchStamped m_measured_cf;

    #endif
    
    string m_nameSpace;

    bool m_is_servo_cp = false;
    bool m_is_servo_jp = false;
    bool m_is_servo_cf = false;

    cTransform m_servo_cp;
    vector<double> m_servo_jp;
    vector<double> m_servo_cf = vector<double>(6);

    
};


#endif //AFCRTK_INTERFACE_H