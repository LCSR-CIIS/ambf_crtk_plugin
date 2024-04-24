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

#include "CRTK_simulator_plugin.h"

Interface::Interface(string ifname){
    m_name = ifname;
    crtkInterface = new afCRTKInterface(ifname);
}

afCRTKSimulatorPlugin::afCRTKSimulatorPlugin(){
    cout << "/*********************************************" << endl;
    cout << "/* AMBF Plugin for CRTK Interface" << endl;
    cout << "/*********************************************" << endl;
}

int afCRTKSimulatorPlugin::init(int argc, char** argv, const afWorldPtr a_afWorld){
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
    string config_filepath = var_map["conf"].as<string>();

    // Define path
    string file_path = __FILE__;
    m_current_filepath = file_path.substr(0, file_path.rfind("/"));

    // Store Pointer for the world
    m_worldPtr = a_afWorld;

    // Improve the constratint
    m_worldPtr->m_bulletWorld->getSolverInfo().m_erp = 1.0;  
    m_worldPtr->m_bulletWorld->getSolverInfo().m_erp2 = 1.0; 
    
    // When config file is defined
    if(!config_filepath.empty()){
        return readConfigFile(config_filepath);
    }

    // If the configuration file is not defined pop error
    else{
        cerr << "WARNING! NO configuration file specified." << endl;
        // Load every Model/Object in the world
        // ModelMap: map<string, afModelPtr>
        afModelMap* map = m_worldPtr->getModelMap();
        for (auto it=map->begin(); it != map->end(); it++){
        
            //ChildrenMap: map<map<afType, map<string, afBaseObject*> >
            afChildrenMap::iterator cIt;
            afChildrenMap* childrenMap = it->second->getChildrenMap();
                        for(cIt = childrenMap->begin(); cIt != childrenMap->end(); ++cIt){   
                for (auto it_child=cIt->second.begin(); it_child != cIt->second.end(); ++it_child){
                    string wholeName = it_child->first;
                    
                    string objectName;
                    string ns;
                    vector<string> v;
                    boost::split(v, wholeName, boost::is_any_of("/")); 

                    if (v.size() ==4){
                        objectName = v[3].substr(v[3].find(" ")+1);
                        ns = "CRTK";
                    }
                    else if (v.size() > 4){
                        vector<string> v1;
                        boost::split(v1, wholeName, boost::is_any_of(" ")); 
                        if (v1.size() > 1){
                            objectName = wholeName.substr(wholeName.find(" ")+1);
                        }
                        else{
                            objectName = v[4];
                        }
                        ns = "CRTK/" + v[3];
                    }
                    else
                        return -1;
                    
                    Interface* interface;
                    if (m_namespaces.find(ns) == m_namespaces.end()){
                        // not found
                        interface = new Interface(ns);
                        m_namespaces[ns] = interface;
                        m_interface.push_back(interface);
                    } 
                    else{
                        // found
                        interface = m_namespaces[ns];
                    }

                    if (it_child->second->getType() == afType::RIGID_BODY){
                        afRigidBodyPtr rigidBodyPtr = m_worldPtr->getRigidBody(wholeName);
                        objectName = regex_replace(objectName, regex{" "}, string{"_"});
                        interface->crtkInterface->add_measured_cp(objectName);
                        interface->crtkInterface->add_measured_cf(objectName);
                        interface->crtkInterface->add_servo_cp(objectName);
                        interface->crtkInterface->add_servo_cf(objectName);
                        interface->m_measuredCPRBsPtr.push_back(rigidBodyPtr); 
                        interface->m_measuredCFRBsPtr.push_back(rigidBodyPtr); 
                        interface->m_servoCPRBsPtr.push_back(rigidBodyPtr); 
                        interface->m_servoCFRBsPtr.push_back(rigidBodyPtr); 
                    }

                    if (it_child->second->getType() == afType::JOINT){
                        afJointPtr jointPtr = m_worldPtr->getJoint(wholeName);
                        objectName = regex_replace(objectName, regex{" "}, string{"_"});
                        interface->m_measuredJointsPtr.push_back(jointPtr);
                        interface->m_servoJointsPtr.push_back(jointPtr);
                    }

                    if (it_child->second->getType() == afType::LIGHT){
                        afBaseObjectPtr objectPtr = m_worldPtr->getLight(wholeName);
                        objectName = regex_replace(objectName, regex{" "}, string{"_"});
                        interface->m_measuredObjectPtr.push_back(objectPtr);
                        interface->m_servoObjectPtr.push_back(objectPtr);
                        interface->crtkInterface->add_measured_cp(objectName);
                        interface->crtkInterface->add_servo_cp(objectName);
                    }

                    if (it_child->second->getType() == afType::CAMERA){
                        afBaseObjectPtr objectPtr = m_worldPtr->getCamera(wholeName);
                        objectName = regex_replace(objectName, regex{" "}, string{"_"});
                        interface->m_measuredObjectPtr.push_back(objectPtr);
                        interface->m_servoObjectPtr.push_back(objectPtr);
                        interface->crtkInterface->add_measured_cp(objectName);
                        interface->crtkInterface->add_servo_cp(objectName);
                    }
                }
            }
        }
        vector<string> jointNames;
        for (size_t i = 0; i < m_interface.size(); i ++){
            if(m_interface[i]->m_measuredJointsPtr.size() > 0){
                for (size_t j = 0; j < m_interface[i]->m_measuredJointsPtr.size(); j++){
                    jointNames.push_back(getNamefromPtr((afBaseObjectPtr)m_interface[i]->m_measuredJointsPtr[j]));
                }
                m_interface[i]->crtkInterface->add_measured_js("", jointNames);
                m_interface[i]->crtkInterface->add_servo_jp("");
            }
            jointNames.clear();
        }
        cerr << "INFO! Initialization Successfully Finished!!" << endl;
        return 1;
    }
}

void afCRTKSimulatorPlugin::keyboardUpdate(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods){

}

void afCRTKSimulatorPlugin::graphicsUpdate(){

}

void afCRTKSimulatorPlugin::physicsUpdate(double dt){
    // Loop for all the interface
    for (size_t index = 0; index < m_interface.size(); index ++){
        // measured_cp
        if (m_interface[index]->m_measuredCPRBsPtr.size() > 0){
            for (size_t i = 0; i < m_interface[index]->m_measuredCPRBsPtr.size(); i++){
                cTransform measured_cp = m_interface[index]->m_measuredCPRBsPtr[i]->getLocalTransform();
                m_interface[index]->crtkInterface->measured_cp(measured_cp, getNamefromPtr((afBaseObjectPtr)m_interface[index]->m_measuredCPRBsPtr[i]));
            }
        }

        // measured_cp
        if (m_interface[index]->m_measuredObjectPtr.size() > 0){
            for (size_t i = 0; i < m_interface[index]->m_measuredObjectPtr.size(); i++){
                cTransform measured_cp = m_interface[index]->m_measuredObjectPtr[i]->getLocalTransform();
                m_interface[index]->crtkInterface->measured_cp(measured_cp, getNamefromPtr((afBaseObjectPtr)m_interface[index]->m_measuredObjectPtr[i]));
            }
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
        if (m_interface[index]->m_measuredCFRBsPtr.size() > 0){
            for (size_t i = 0; i < m_interface[index]->m_measuredCFRBsPtr.size(); i++){
                btVector3 bt_measured_cf = m_interface[index]->m_measuredCFRBsPtr[i]->m_estimatedForce;
                vector<double> measured_cf{bt_measured_cf.getX(),bt_measured_cf.getY(),bt_measured_cf.getZ(),0,0,0};
                m_interface[index]->crtkInterface->measured_cf(measured_cf);
            }
        }

        // servo_cp
        if (m_interface[index]->m_servoCPRBsPtr.size() > 0){
            cTransform servo_cp;
            for (size_t i = 0; i < m_interface[index]->m_servoCPRBsPtr.size(); i++){
                if(m_interface[index]->crtkInterface->servo_cp(servo_cp)){
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
                    m_interface[index]->m_servoCPRBsPtr[i]->m_bulletRigidBody->getMotionState()->setWorldTransform(Tcommand);
                    m_interface[index]->m_servoCPRBsPtr[i]->m_bulletRigidBody->setWorldTransform(Tcommand);
                }   
            }
        }

        // servo_cp
        if (m_interface[index]->m_servoObjectPtr.size() > 0){
            cTransform servo_cp;
            for (size_t i = 0; i < m_interface[index]->m_servoObjectPtr.size(); i++){
                if(m_interface[index]->crtkInterface->servo_cp(servo_cp)){
                    m_interface[index]->m_servoObjectPtr[i]->setLocalTransform(servo_cp);
                }   
            }
        }

        // servo_jp
        if (m_interface[index]->m_servoJointsPtr.size() > 0){
            vector<double> servo_jp;
            if(m_interface[index]->crtkInterface->servo_jp(servo_jp)){
                for  (size_t i = 0; i < m_interface[index]->m_servoJointsPtr.size(); i++){
                    m_interface[index]->m_servoJointsPtr[i]->commandPosition(servo_jp[i]);
                }
            }
        }

        // servo_cf
        if (m_interface[index]->m_servoCFRBsPtr.size() > 0){
            for (size_t i = 0; i < m_interface[index]->m_servoCFRBsPtr.size(); i++){
                vector<double> servo_cf;
                if(m_interface[index]->crtkInterface->servo_cf(servo_cf)){
                    m_interface[index]->m_servoCFRBsPtr[i]->applyForce(cVector3d(servo_cf[0], servo_cf[1], servo_cf[2]));
                    m_interface[index]->m_servoCFRBsPtr[i]->applyTorque(cVector3d(servo_cf[3], servo_cf[4], servo_cf[5]));
                }
            }
        }
    }
}

int afCRTKSimulatorPlugin::readConfigFile(string config_filepath){
    YAML::Node node = YAML::LoadFile(config_filepath);

    m_num = node["interface"].size();
    for (size_t i = 0; i < m_num; i++){
        string ifname = node["interface"][i].as<string>();
        Interface* interface = new Interface(ifname);
        m_interface.push_back(interface);
        
        if (InitInterface(node, interface, ifname) == -1)
            return -1;
    }
    return 1;         
}

int afCRTKSimulatorPlugin::InitInterface(YAML::Node& node, Interface* interface, string ifname){
    
    if (node[ifname]["measured_cp"]){
        if(node[ifname]["measured_cp"]["rigidbody"]){
            cerr << "[INFO!] Adding measured_cp ... " << endl;
            for (int j = 0; j < node[ifname]["measured_cp"]["rigidbody"].size(); j++){
                string rigidName = node[ifname]["measured_cp"]["rigidbody"][j].as<string>();
                afRigidBodyPtr rigidBodyPtr = m_worldPtr->getRigidBody(rigidName);

                if(!rigidBodyPtr){
                    cerr << ">> ERROR!! No RigidBody Named " << node[ifname]["measured_cp"]["rigidbody"][j].as<string>() << " for measured_cp" << endl;
                    return -1;
                }
                interface->m_measuredCPRBsPtr.push_back(rigidBodyPtr);
                rigidName = getNamefromPtr((afBaseObjectPtr)rigidBodyPtr);
                

                if(node[ifname]["measured_cp"]["namespace"] && node[ifname]["measured_cp"]["rigidbody"].size() == node[ifname]["measured_cp"]["namespace"].size())
                    interface->crtkInterface->add_measured_cp(node[ifname]["measured_cp"]["namespace"][j].as<string>() + '/' + rigidName);
                else
                    interface->crtkInterface->add_measured_cp(rigidName);           
            }
        }

        else{  
            cerr << ">> ERROR!! No RigidBody specified for measured_cp" << endl;
            return -1;
        }
    }
    if (node[ifname]["measured_js"]){
        vector<string> jointNames;
        cerr << "[INFO!] Adding measured_js ... " << endl;
        for (size_t j = 0; j < node[ifname]["measured_js"]["joints"].size(); j++){
            string jointName = node[ifname]["measured_js"]["joints"][j].as<string>();
            interface->m_measuredJointsPtr.push_back(m_worldPtr->getJoint(jointName));
            jointNames.push_back(jointName);
        }

        if(node[ifname]["measured_js"]["namespace"])
            interface->crtkInterface->add_measured_js(node[ifname]["measured_js"]["namespace"].as<string>(), jointNames);
        else
            interface->crtkInterface->add_measured_js("", jointNames);
    }

    if (node[ifname]["measured_cf"]){
        if(node[ifname]["measured_cf"]["rigidbody"]){
            cerr << "[INFO!] Adding measured_cf ... " << endl;
            for (int j = 0; j < node[ifname]["measured_cf"]["rigidbody"].size(); j++){
                string rigidName = node[ifname]["measured_cf"]["rigidbody"][j].as<string>();
                afRigidBodyPtr rigidBodyPtr = m_worldPtr->getRigidBody(rigidName);                
                
                if(!rigidBodyPtr){
                    cerr << ">> ERROR!! No RigidBody Named " << node[ifname]["measured_cf"]["rigidbody"][j].as<string>() << " for measured_cf" << endl;
                    return -1;
                }
                interface->m_measuredCFRBsPtr.push_back(rigidBodyPtr);
                rigidName = getNamefromPtr((afBaseObjectPtr)rigidBodyPtr);

                if(node[ifname]["measured_cf"]["namespace"] && node[ifname]["measured_cf"]["rigidbody"].size() == node[ifname]["measured_cf"]["namespace"].size())
                    interface->crtkInterface->add_measured_cf(node[ifname]["measured_cf"]["namespace"][j].as<string>()+ '/' + rigidName);
                else
                    interface->crtkInterface->add_measured_cf(rigidName);
            }
        }

        else{  
            cerr << ">> ERROR!! No RigidBody specified for measured_cf" << endl;
            return -1;
        }
    }

    if (node[ifname]["servo_cp"]){
        if(node[ifname]["servo_cp"]["rigidbody"]){
            cerr << "[INFO!] Adding servo_cp ... " << endl;
            for (int j = 0; j < node[ifname]["servo_cp"]["rigidbody"].size(); j++){
                string rigidName = node[ifname]["servo_cp"]["rigidbody"][j].as<string>();
                afRigidBodyPtr rigidBodyPtr = m_worldPtr->getRigidBody(rigidName);
                if(!rigidBodyPtr){
                    cerr << ">> ERROR!! No RigidBody Named " << node[ifname]["servo_cp"]["rigidbody"][j].as<string>() << " for servo_cp" << endl;
                    return -1;
                }

                interface->m_servoCPRBsPtr.push_back(rigidBodyPtr);
            
                rigidName = getNamefromPtr((afBaseObjectPtr)rigidBodyPtr);

                if(node[ifname]["servo_cp"]["namespace"]&& node[ifname]["servo_cp"]["rigidbody"].size() == node[ifname]["servo_cp"]["namespace"].size())
                    interface->crtkInterface->add_servo_cp(node[ifname]["servo_cp"]["rigidbody"][j]["namespace"].as<string>() + '/' = rigidName);
                else
                    interface->crtkInterface->add_servo_cp(rigidName);
            }

        }

        else{  
            cerr << ">> ERROR!! No RigidBody specified for servo_cp" << endl;
            return -1;
        }
    }

    if (node[ifname]["servo_jp"]){
        cerr << "[INFO!] Adding servo_jp ... " << endl;
        vector<string> jointNames;
        for (size_t j = 0; j < node[ifname]["servo_jp"]["joints"].size(); j++){
            string jointName = node[ifname]["servo_jp"]["joints"][j].as<string>();
            jointNames.push_back(jointName);
            interface->m_servoJointsPtr.push_back(m_worldPtr->getJoint(jointName));
        }

        if(node[ifname]["servo_jp"]["namespace"])
            interface->crtkInterface->add_servo_jp(node[ifname]["servo_jp"]["namespace"].as<string>());
        else
            interface->crtkInterface->add_servo_jp("");
    }

    if (node[ifname]["servo_cf"]){
        if(node[ifname]["servo_cf"]["rigidbody"]){
            cerr << "[INFO!] Adding servo_cf ... " << endl;
            for (int j = 0; j < node[ifname]["servo_cf"]["rigidbody"].size(); j++){
                string rigidName = node[ifname]["servo_cf"]["rigidbody"].as<string>();
                afRigidBodyPtr rigidBodyPtr = m_worldPtr->getRigidBody(rigidName);
                if(!rigidBodyPtr){
                    cerr << ">> ERROR!! No RigidBody Named " << node[ifname]["servo_cf"]["rigidbody"].as<string>() << " for servo_cf" << endl;
                    return -1;
                }
                interface->m_servoCFRBsPtr.push_back(rigidBodyPtr);
                
                rigidName = getNamefromPtr((afBaseObjectPtr)rigidBodyPtr);
                if(node[ifname]["servo_cf"]["namespace"])
                    interface->crtkInterface->add_servo_cf(node[ifname]["servo_cf"]["namespace"].as<string>() + '/' + rigidName);
                else
                    interface->crtkInterface->add_servo_cf(rigidName);
            }
        }

        else{  
            cerr << ">> ERROR!! No RigidBody specified for servo_cp" << endl;
            return -1;
        }
    }

    cerr << "Successfully initialized interface" << endl;
    return 1;

}

string getNamefromPtr(afBaseObjectPtr baseBodyPtr){
    // Get Namespace
    string ns = baseBodyPtr->getAttributes()->m_identificationAttribs.m_namespace;
    // cerr << ns.erase(0,9) << endl; // Erase "/ambf/env"
    ns = ns.erase(0,10);
    
    string baseName = baseBodyPtr->getAttributes()->m_identifier; // BODY name_of_rigidBody
    vector<string> v;
    boost::split(v, baseName, boost::is_any_of(" ")); 

    if (v.size() > 1){
        baseName.erase(0,v[0].length()+1); //Remove BODY
        baseName = baseName;//ns + baseName;
    }
    baseName = regex_replace(baseName, regex{" "}, string{"_"});
    return baseName;
}

void afCRTKSimulatorPlugin::reset(){

}

bool afCRTKSimulatorPlugin::close(){
    return true;
}