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

#include "CRTK_plugin.h"


afCRTKPlugin::afCRTKPlugin(){
    cout << "/*********************************************" << endl;
    cout << "/* AMBF Plugin for CRTK Interface" << endl;
    cout << "/*********************************************" << endl;
}

int afCRTKPlugin::init(int argc, char** argv, const afWorldPtr a_afWorld){
    p_opt::options_description cmd_opts("Robot control Command Line Options");
    cmd_opts.add_options()
            ("info", "Show Info")
            ("conf", p_opt::value<string>()->default_value(""), "Name of specfile for spacenav control");

    p_opt::variables_map var_map;
    p_opt::store(p_opt::command_line_parser(argc, argv).options(cmd_opts).allow_unregistered().run(), var_map);
    p_opt::notify(var_map);

    if(var_map.count("info")){
        std::cout<< cmd_opts << std::endl;
        return -1;
    }

    // Loading options 
    string config_filepath = var_map["registration_config"].as<string>();

    // Define path
    string file_path = __FILE__;
    m_current_filepath = file_path.substr(0, file_path.rfind("/"));

    // Store Pointer for the world
    m_worldPtr = a_afWorld;
    
    // When config file is defined
    if(!config_filepath.empty()){
        return readConfigFile(config_filepath);
    }

    // If the configuration file is not defined pop error
    else{
        cerr << "ERROR! NO configuration file specified." << endl;
        return -1;
    }
}

void afCRTKPlugin::keyboardUpdate(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods){

}

void afCRTKPlugin::graphicsUpdate(){

}

void afCRTKPlugin::physicsUpdate(double dt){
    // Loop for all the interface
    for (size_t index = 0; index < m_interface.size(); index ++){
        // measured_cp
        if (m_interface[index]->m_measuredCPRB){
            cTransform measured_cp = m_interface[index]->m_measuredCPRB->getLocalTransform();
            m_interface[index]->crtkInterface->measured_cp(measured_cp);
        }

        // measured_js
        if (m_interface[index]->m_measuredJointsPtr.size() > 0){
            vector<double> measured_js;
            for  (size_t i = 0; i < m_interface[index]->m_measuredJointsPtr.size(); i++){
                double jointPos = m_interface[index]->m_measuredJointsPtr[i]->getPosition();
                measured_js.push_back(jointPos);
            }
            m_interface[index]->crtkInterface->measured_js(measured_js);
        }

        // measured_cf
        if (m_interface[index]->m_measuredCFRB){
            btVector3 bt_measured_cf = m_interface[index]->m_measuredCFRB->m_estimatedForce;
            vector<double> measured_cf{bt_measured_cf.getX(),bt_measured_cf.getY(),bt_measured_cf.getZ() };
            m_interface[index]->crtkInterface->measured_cf(measured_cf);
        }

        // servo_cp
        if (m_interface[index]->m_servoCPRB){

            cTransform servo_cp = m_interface[index]->crtkInterface->servo_cp();
            // Change to btVector and Matrix
            btTransform trans;
            btVector3 btTrans;
            
            btTrans.setValue(servo_cp.getLocalPos().x(), servo_cp.getLocalPos().y(), servo_cp.getLocalPos().z());     
            trans.setOrigin(btTrans);
            cQuaternion quat;
            quat.fromRotMat(servo_cp.getLocalRot());
            btQuaternion btRot(quat.x,quat.y,quat.z,quat.w);
            trans.setRotation(btRot);

            btTransform Tcommand;
            Tcommand = trans;
            m_interface[index]->m_servoCPRB->m_bulletRigidBody->getMotionState()->setWorldTransform(Tcommand);
            m_interface[index]->m_servoCPRB->m_bulletRigidBody->setWorldTransform(Tcommand);
        }

        // servo_jp
        if (m_interface[index]->m_servoJointsPtr.size() > 0){
            vector<double> servo_jp = m_interface[index]->crtkInterface->servo_jp();
            for  (size_t i = 0; i < m_interface[index]->m_servoJointsPtr.size(); i++){
                m_interface[index]->m_servoJointsPtr[i]->commandPosition(servo_jp[i]);
            }
        }

        // servo_cf
        if (m_interface[index]->m_servoCFRB){
            vector<double> servo_cf = m_interface[index]->crtkInterface->servo_cf();
            m_interface[index]->m_servoCFRB->applyForce(cVector3d(servo_cf[0], servo_cf[1], servo_cf[2]));
            m_interface[index]->m_servoCFRB->applyTorque(cVector3d(servo_cf[3], servo_cf[4], servo_cf[5]));
        }
    }
    

}

int afCRTKPlugin::readConfigFile(string config_filepath){
    YAML::Node node = YAML::LoadFile(config_filepath);

    m_num = node["interface"].size();
    for (size_t i = 0; i < m_num; i++){
        string ifname = node["interface"][i].as<string>();
        Interface* interface = new Interface();
        interface->crtkInterface = new CRTKInterface(ifname);
        m_interface.push_back(interface);
        
        if (InitInterface(node, interface, ifname) == -1)
            return -1;
    }
    return 1;
}

int afCRTKPlugin::InitInterface(YAML::Node& node, Interface* interface, string ifname){
    if (node[ifname]["measured_cp"]){
        if(node[ifname]["measured_cp"]["namespace"])
            interface->crtkInterface->add_measured_cp(node[ifname]["measured_cp"]["namespace"].as<string>());
        else
            interface->crtkInterface->add_measured_cp("");

        if(node[ifname]["measured_cp"]["rigidbody"]){
            interface->m_measuredCPRB = m_worldPtr->getRigidBody(node[ifname]["measured_cp"]["rigidbody"].as<string>());
            
            if(!interface->m_measuredCPRB){
                cerr << ">> ERROR!! No RigidBody Named" << node[ifname]["measured_cp"]["rigidbody"].as<string>() << " for measured_cp" << endl;
                return -1;
            }
        }

        else{  
            cerr << ">> ERROR!! No RigidBody specified for measured_cp" << endl;
            return -1;
        }
    }

    else if (node[ifname]["measured_js"]){
        if(node[ifname]["measured_js"]["namespace"])
            interface->crtkInterface->add_measured_js(node[ifname]["measured_js"]["namespace"].as<string>());
        else
            interface->crtkInterface->add_measured_js("");

        for (size_t j = 0; j < node[ifname]["measured_js"]["joints"].size(); j++){
            interface->m_measuredJointsPtr.push_back(m_worldPtr->getJoint(node[ifname]["measured_js"]["joints"][j].as<string>()));
        }
    }

    if (node[ifname]["measured_cf"]){
        if(node[ifname]["measured_cf"]["namespace"])
            interface->crtkInterface->add_measured_cf(node[ifname]["measured_cf"]["namespace"].as<string>());
        else
            interface->crtkInterface->add_measured_cf("");

        if(node[ifname]["measured_cf"]["rigidbody"]){
            interface->m_measuredCFRB = m_worldPtr->getRigidBody(node[ifname]["measured_cf"]["rigidbody"].as<string>());
            
            if(!interface->m_measuredCFRB){
                cerr << ">> ERROR!! No RigidBody Named" << node[ifname]["measured_cf"]["rigidbody"].as<string>() << " for measured_cf" << endl;
                return -1;
            }
        }

        else{  
            cerr << ">> ERROR!! No RigidBody specified for measured_cf" << endl;
            return -1;
        }
    }

    if (node[ifname]["servo_cp"]){
        if(node[ifname]["servo_cp"]["namespace"])
            interface->crtkInterface->add_servo_cp(node[ifname]["servo_cp"]["namespace"].as<string>());
        else
            interface->crtkInterface->add_servo_cp("");

        if(node[ifname]["servo_cp"]["rigidbody"]){
            interface->m_servoCPRB = m_worldPtr->getRigidBody(node[ifname]["servo_cp"]["rigidbody"].as<string>());
            
            if(!interface->m_servoCPRB){
                cerr << ">> ERROR!! No RigidBody Named" << node[ifname]["servo_cp"]["rigidbody"].as<string>() << " for servo_cp" << endl;
                return -1;
            }
        }

        else{  
            cerr << ">> ERROR!! No RigidBody specified for servo_cp" << endl;
            return -1;
        }
    }

    if (node[ifname]["servo_jp"]){
        if(node[ifname]["servo_jp"]["namespace"])
            interface->crtkInterface->add_servo_jp(node[ifname]["servo_jp"]["namespace"].as<string>());
        else
            interface->crtkInterface->add_servo_jp("");

        for (size_t j = 0; j < node[ifname]["servo_jp"]["joints"].size(); j++){
            interface->m_servoJointsPtr.push_back(m_worldPtr->getJoint(node[ifname]["servo_js"]["joints"][j].as<string>()));
        }
    }

    if (node[ifname]["servo_cf"]){
        if(node[ifname]["servo_cf"]["namespace"])
            interface->crtkInterface->add_servo_cf(node[ifname]["servo_cf"]["namespace"].as<string>());
        else
            interface->crtkInterface->add_servo_cf("");

        if(node[ifname]["servo_cf"]["rigidbody"]){
            interface->m_servoCFRB = m_worldPtr->getRigidBody(node[ifname]["servo_cf"]["rigidbody"].as<string>());
            
            if(!interface->m_servoCFRB){
                cerr << ">> ERROR!! No RigidBody Named" << node[ifname]["servo_cf"]["rigidbody"].as<string>() << " for servo_cf" << endl;
                return -1;
            }
        }

        else{  
            cerr << ">> ERROR!! No RigidBody specified for servo_cp" << endl;
            return -1;
        }
    }

    return 1;

}

void afCRTKPlugin::reset(){

}

bool afCRTKPlugin::close(){
    return true;
}