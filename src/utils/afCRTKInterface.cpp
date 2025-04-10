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

#include "afCRTKInterface.h"

afCRTKInterface::afCRTKInterface(string a_namespace){
    init(a_namespace);
}


afCRTKInterface::~afCRTKInterface(){
}


void afCRTKInterface::init(string a_namespace){
    m_rosNode = afROSNode::getNodeAndRegister(a_namespace);
    m_nameSpace = a_namespace;
    cout << "Name Space:" << m_nameSpace << endl;
}


void afCRTKInterface::add_allInterface(string a_namespace){
    ambf_ral::create_publisher<AMBF_RAL_MSG(geometry_msgs, PoseStamped)>
      (m_setpointCPPub, m_rosNode, m_nameSpace + "/setpoint_cp", 1, false);
    ambf_ral::create_publisher<AMBF_RAL_MSG(geometry_msgs, PoseStamped)>
      (m_measuredCPPub, m_rosNode, m_nameSpace + "/measured_cp", 1, false);
    ambf_ral::create_publisher<AMBF_RAL_MSG(sensor_msgs, JointState)>
      (m_measuredJSPub, m_rosNode, m_nameSpace + "/measured_js", 1, false);
    ambf_ral::create_publisher<AMBF_RAL_MSG(geometry_msgs, WrenchStamped)>
      (m_measuredCFPub, m_rosNode, m_nameSpace + "/measured_cf", 1, false);

    ambf_ral::create_subscriber<AMBF_RAL_MSG(geometry_msgs, PoseStamped), afCRTKInterface>
      (m_servoCPSub, m_rosNode, m_nameSpace + "/servo_cp", 1, &afCRTKInterface::servo_CPCallback, this);
    ambf_ral::create_subscriber<AMBF_RAL_MSG(sensor_msgs, JointState), afCRTKInterface>
      (m_servoJPSub, m_rosNode, m_nameSpace + "/servo_jp", 1, &afCRTKInterface::servo_JPCallback, this);
    ambf_ral::create_subscriber<AMBF_RAL_MSG(geometry_msgs, WrenchStamped), afCRTKInterface>
      (m_servoCFSub, m_rosNode, m_nameSpace + "/servo_cf", 1, &afCRTKInterface::servo_CFCallback, this);
    // m_servoCPSub = m_rosNode->subscribe(m_nameSpace + "/servo_cp", 1, &afCRTKInterface::servo_CPCallback, this);
    // m_servoJPSub = m_rosNode->subscribe(m_nameSpace + "/servo_jp", 1, &afCRTKInterface::servo_JPCallback, this);
    // m_servoCFSub = m_rosNode->subscribe(m_nameSpace + "/servo_cf", 1 , &afCRTKInterface::servo_CFCallback, this);
}

void afCRTKInterface::add_operating_state(string a_namespace){
    string baseName;
    if(a_namespace == ""){
        baseName = m_nameSpace;
    }
    else{
        baseName = a_namespace;
    }
    ambf_ral::create_publisher<AMBF_RAL_MSG(crtk_msgs, OperatingState)>
      (m_operatingStatePub, m_rosNode, baseName + "/operating_state", 1, false);
      
    // m_operatingStatePub = m_rosNode->advertise<crtk_msgs::OperatingState>(baseName + "/operating_state", 1);
}

void afCRTKInterface::add_state_command(string a_namespace){
    string baseName;
    if(a_namespace == ""){
        baseName = m_nameSpace;
    }
    else{
        baseName = a_namespace;
    }
    ambf_ral::create_publisher<AMBF_RAL_MSG(crtk_msgs, StringStamped)>
      (m_stateCommandPub, m_rosNode, baseName + "/state_command", 1, false);
      
    // m_operatingStatePub = m_rosNode->advertise<crtk_msgs::OperatingState>(baseName + "/operating_state", 1);
}

void afCRTKInterface::add_measured_cp(string a_namespace){
    string baseName;
    if(a_namespace == ""){
        baseName = m_nameSpace;
    }
    else{
        baseName = a_namespace;
    }
    ambf_ral::create_publisher<AMBF_RAL_MSG(geometry_msgs, PoseStamped)>
      (m_measuredCPPub, m_rosNode, baseName + "/measured_cp", 1, false);

    m_measuredCPPubMap[baseName] = m_measuredCPPub;
    
    // m_measuredCPPub = m_rosNode->advertise<geometry_msgs::PoseStamped>(baseName + "/measured_cp", 1);
    // m_measuredCPPubMap[a_namespace] = m_rosNode->advertise<geometry_msgs::PoseStamped>(baseName + "/measured_cp", 1);
}

void afCRTKInterface::add_setpoint_cp(string a_namespace){
    string baseName;
    if(a_namespace == ""){
        baseName = m_nameSpace;
    }
    else{
        baseName = a_namespace;
    }
    ambf_ral::create_publisher<AMBF_RAL_MSG(geometry_msgs, PoseStamped)>
      (m_setpointCPPub, m_rosNode, baseName + "/setpoint_cp", 1, false);

    m_setpointCPPubMap[baseName] = m_setpointCPPub;
    
    // m_measuredCPPub = m_rosNode->advertise<geometry_msgs::PoseStamped>(baseName + "/measured_cp", 1);
    // m_measuredCPPubMap[a_namespace] = m_rosNode->advertise<geometry_msgs::PoseStamped>(baseName + "/measured_cp", 1);
}


void afCRTKInterface::add_measured_js(string a_namespace, vector<string> jointNames){
    string baseName;
    if(a_namespace == ""){
        baseName = m_nameSpace;
    }
    else{
        baseName = a_namespace;
    }
    ambf_ral::create_publisher<AMBF_RAL_MSG(sensor_msgs, JointState)>
      (m_measuredJSPub, m_rosNode, baseName + "/measured_js", 1, false);
    // m_measuredJSPub = m_rosNode->advertise<sensor_msgs::JointState>(baseName+ "/measured_js", 1);

    m_measuredJSPubMap[baseName] = m_measuredJSPub;
}


void afCRTKInterface::add_measured_cf(string a_namespace){
    string baseName;
    if(a_namespace == ""){
        baseName = m_nameSpace;
    }
    else{
        baseName = a_namespace;
    }
    ambf_ral::create_publisher<AMBF_RAL_MSG(geometry_msgs, WrenchStamped)>
      (m_measuredCFPub, m_rosNode, baseName + "/measured_cf", 1, false);
    // m_measuredCFPub = m_rosNode->advertise<geometry_msgs::WrenchStamped>(baseName + "/measured_cf", 1);
    m_measuredCFPubMap[baseName] = m_measuredCFPub; 
}


void afCRTKInterface::add_servo_cp(string a_namespace){
    string baseName;
    if(a_namespace == ""){
        baseName = m_nameSpace;
    }
    else{
        baseName = a_namespace;
    }

    ambf_ral::create_subscriber<AMBF_RAL_MSG(geometry_msgs, PoseStamped), afCRTKInterface>
      (m_servoCPSub, m_rosNode, baseName + "/servo_cp", 1, &afCRTKInterface::servo_CPCallback, this);
    // m_servoCPSub = m_rosNode->subscribe(baseName + "/servo_cp", 1, &afCRTKInterface::servo_CPCallback, this);

    m_servoCPSubMap[baseName] = m_servoCPSub;
}


void afCRTKInterface::add_servo_jp(string a_namespace){
    string baseName;
    if(a_namespace == ""){
        baseName = m_nameSpace;
    }
    else{
        baseName = a_namespace;
    }
    ambf_ral::create_subscriber<AMBF_RAL_MSG(sensor_msgs, JointState), afCRTKInterface>
      (m_servoJPSub, m_rosNode, baseName + "/servo_jp", 1, &afCRTKInterface::servo_JPCallback, this);
    // m_servoJPSub = m_rosNode->subscribe(baseName + "/servo_jp", 1, &afCRTKInterface::servo_JPCallback, this);
    m_servoJPSubMap[baseName] = m_servoJPSub;
}


void afCRTKInterface::add_servo_cf(string a_namespace){
    string baseName;
    if(a_namespace == ""){
        baseName = m_nameSpace;
    }
    else{
        baseName = a_namespace;
    }
    ambf_ral::create_subscriber<AMBF_RAL_MSG(geometry_msgs, WrenchStamped), afCRTKInterface>
      (m_servoCFSub, m_rosNode, baseName + "/servo_cf", 1, &afCRTKInterface::servo_CFCallback, this);
    // m_servoCFSub = m_rosNode->subscribe(baseName + "/servo_cf", 1 , &afCRTKInterface::servo_CFCallback, this);
    m_servoCFSubMap[baseName] = m_servoCFSub;
}

   
void afCRTKInterface::servo_CPCallback(AMBF_RAL_MSG_PTR(geometry_msgs, PoseStamped) msg){
    m_servo_cp.setLocalPos(cVector3d(msg->pose.position.x,
                                        msg->pose.position.y,
                                        msg->pose.position.z));
    cQuaternion rot(msg->pose.orientation.w,
                    msg->pose.orientation.x,
                    msg->pose.orientation.y,
                    msg->pose.orientation.z);
    cMatrix3d rotM;
    rot.toRotMat(rotM);

    m_servo_cp.setLocalRot(rotM);
    m_is_servo_cp = true;
}


void afCRTKInterface::servo_JPCallback(AMBF_RAL_MSG_PTR(sensor_msgs, JointState) msg){
    // if (msg->position.size() != m_servo_jp.size()){
    //     cerr << "ERROR! IN Measured JS, JOINT LENGTH MUST BE " << m_measured_js.name.size() << endl;
    //     return;
    // }
    m_servo_jp = msg->position;
    m_is_servo_jp = true;
}


void afCRTKInterface::servo_CFCallback(AMBF_RAL_MSG_PTR(geometry_msgs, WrenchStamped) msg){
    m_servo_cf[0] = msg->wrench.force.x;
    m_servo_cf[1] = msg->wrench.force.y;
    m_servo_cf[2] = msg->wrench.force.z;
    m_servo_cf[3] = msg->wrench.torque.x;
    m_servo_cf[4] = msg->wrench.torque.y;
    m_servo_cf[5] = msg->wrench.torque.z;
    m_is_servo_cf = true;
}


bool afCRTKInterface::servo_cp(cTransform& cp){
    cp = m_servo_cp;
    return m_is_servo_cp;
}


bool afCRTKInterface::servo_jp(vector<double> & jp){
    jp = m_servo_jp;
    return m_is_servo_jp;
}


bool afCRTKInterface::servo_cf(vector<double> & cf){
    cf = m_servo_cf;
    return m_is_servo_cf;
}

void afCRTKInterface::run_operating_state(){
    ambf_ral::spin_some(m_rosNode);
    m_operatingState.state = "ENABLED";
    m_operatingState.is_homed = false;
    m_operatingState.is_busy = false;

    #if AMBF_ROS1
        m_operatingStatePub.publish(m_operatingState);
    #elif AMBF_ROS2
        m_operatingStatePub->publish(m_operatingState);
    #endif
}

void afCRTKInterface::run_state_command(){
    m_state_command.string = "enable";
    #if AMBF_ROS1
        m_stateCommandPub.publish(m_state_command);
    #elif AMBF_ROS2
        m_stateCommandPub->publish(m_state_command);
    #endif

}

void afCRTKInterface::measured_cp(cTransform &trans, string name){
    
    m_measured_cp.pose.position.x = trans.getLocalPos().x();
    m_measured_cp.pose.position.y = trans.getLocalPos().y();
    m_measured_cp.pose.position.z = trans.getLocalPos().z();

    cQuaternion rot;
    rot.fromRotMat(trans.getLocalRot());
    m_measured_cp.pose.orientation.x = rot.x;
    m_measured_cp.pose.orientation.y = rot.y;
    m_measured_cp.pose.orientation.z = rot.z;
    m_measured_cp.pose.orientation.w = rot.w;

    #if AMBF_ROS1
    if (name == "default"){
        m_measuredCPPub.publish(m_measured_cp);
    }
    else{
        m_measuredCPPubMap[name].publish(m_measured_cp);
    }
    #elif AMBF_ROS2
    if (name == "default"){
        m_measuredCPPub->publish(m_measured_cp);
    }
    else{
        m_measuredCPPubMap[name]->publish(m_measured_cp);
    }
    #endif
}

void afCRTKInterface::setpoint_cp(cTransform &trans, string name){
    
    m_setpoint_cp.pose.position.x = trans.getLocalPos().x();
    m_setpoint_cp.pose.position.y = trans.getLocalPos().y();
    m_setpoint_cp.pose.position.z = trans.getLocalPos().z();

    cQuaternion rot;
    rot.fromRotMat(trans.getLocalRot());
    m_setpoint_cp.pose.orientation.x = rot.x;
    m_setpoint_cp.pose.orientation.y = rot.y;
    m_setpoint_cp.pose.orientation.z = rot.z;
    m_setpoint_cp.pose.orientation.w = rot.w;

    #if AMBF_ROS1
    if (name == "default"){
        m_setpointCPPub.publish(m_setpoint_cp);
    }
    else{
        m_setpointCPPubMap[name].publish(m_setpoint_cp);
    }
    #elif AMBF_ROS2
    if (name == "default"){
        m_setpointCPPub->publish(m_setpoint_cp);
    }
    else{
        m_setpointCPPubMap[name]->publish(m_setpoint_cp);
    }
    #endif
    
}


void afCRTKInterface::measured_js(vector<double>& q, vector<string> jointNames, string name){
    // Initialize the position value for the measured_js values
    m_measured_js.position.clear();
    for (int idx = 0 ; idx < q.size() ; idx++){
        m_measured_js.position.push_back(q[idx]);
    }

    m_measured_js.name = jointNames;

    #if AMBF_ROS1
    if (name == "default"){
        m_measuredJSPub.publish(m_measured_js);
    }
    else{
        m_measuredJSPubMap[name].publish(m_measured_js);
    }  
    #elif AMBF_ROS2
    if (name == "default"){
        m_measuredJSPub->publish(m_measured_js);
    }
    else{
        m_measuredJSPubMap[name]->publish(m_measured_js);
    }  
    #endif
    
}


void afCRTKInterface::measured_cf(vector<double>& force, string name){

    if (force.size() != 6){
        cerr << "ERROR! IN MEASURED_CF, FORCE HAS TO HAVE 6DOF." << endl;
        return;
    }

    m_measured_cf.wrench.force.x = force[0];
    m_measured_cf.wrench.force.y = force[1];
    m_measured_cf.wrench.force.z = force[2];
    m_measured_cf.wrench.torque.x = force[3];
    m_measured_cf.wrench.torque.y = force[4];
    m_measured_cf.wrench.torque.z = force[5];
    #if AMBF_ROS1
    if (name == "default"){
        m_measuredCFPub.publish(m_measured_cf);
    }
    else{
        m_measuredCFPubMap[name].publish(m_measured_cf);
    }  
    #elif AMBF_ROS2
    if (name == "default"){
        m_measuredCFPub->publish(m_measured_cf);
    }
    else{
        m_measuredCFPubMap[name]->publish(m_measured_cf);
    }  
    #endif
    
}
