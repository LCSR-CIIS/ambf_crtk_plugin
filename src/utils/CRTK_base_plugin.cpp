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
    \date      10.10.2024
    
*/
//==============================================================================

#include "CRTK_base_plugin.h"

Interface::Interface(string ifname){
    m_name = ifname;
    crtkInterface = new afCRTKInterface(ifname);
}

afCRTKBasePlugin::afCRTKBasePlugin(){
}

int afCRTKBasePlugin::readConfigFile(string config_filepath){
    YAML::Node node = YAML::LoadFile(config_filepath);

    cerr << "Loading configuration file: " << config_filepath << endl;

    m_numInterface = node["interface"].size();
    for (size_t i = 0; i < m_numInterface; i++){
        string ifname = node["interface"][i].as<string>();
        Interface* interface = new Interface(ifname);
        m_interface.push_back(interface);
        
        if (InitInterface(node, interface) == -1)
            return -1;
    }
    return 1;         
}

int afCRTKBasePlugin::InitInterface(YAML::Node& node, Interface* interface){
    // Add operating state and state command
    interface->crtkInterface->add_operating_state("");
    interface->crtkInterface->add_state_command("");
    string nspace;
    
    if (node[interface->m_name]["measured_cp"]){
        for (size_t i = 0; i < node[interface->m_name]["measured_cp"].size(); i++){
            if(node[interface->m_name]["measured_cp"][i]["rigidbody"]){
                cerr << "[INFO!] Adding measured_cp ... " << endl;
                if (node[interface->m_name]["measured_cp"][i]["reference"]){
                    string objectName = node[interface->m_name]["measured_cp"][i]["reference"].as<string>();
                    interface->m_referenceMeasuredPtr = (afBaseObjectPtr)m_worldPtr->getRigidBody(objectName);
                }

                string rigidName = node[interface->m_name]["measured_cp"][i]["rigidbody"].as<string>();
                afRigidBodyPtr rigidBodyPtr = m_worldPtr->getRigidBody(rigidName);

                if(!rigidBodyPtr){
                    cerr << ">> ERROR!! No RigidBody Named " << node[interface->m_name]["measured_cp"][i]["rigidbody"].as<string>() << " for measured_cp" << endl;
                    return -1;
                }
                
                if(node[interface->m_name]["measured_cp"][i]["namespace"]){
                    nspace = node[interface->m_name]["measured_cp"][i]["namespace"].as<string>();
                    interface->m_measuredCPRBsPtr[nspace] = rigidBodyPtr;

                    if (interface->m_referenceMeasuredPtr){
                        interface->crtkInterface->add_measured_cp(nspace);
                    }
                    interface->crtkInterface->add_measured_cp(nspace + "/local/");
                }
        
                else{
                    rigidName = getNamefromPtr((afBaseObjectPtr)rigidBodyPtr);

                    if (interface->m_referenceMeasuredPtr){
                        nspace = interface->m_name + '/' + rigidName;
                        interface->m_measuredCPRBsPtr[nspace] = rigidBodyPtr;
                        interface->crtkInterface->add_measured_cp(nspace); 
                    }
                    nspace = interface->m_name + "/local/" + rigidName;
                    interface->m_measuredCPRBsPtr[nspace] = rigidBodyPtr;
                    interface->crtkInterface->add_measured_cp(nspace); 
                }
            }

            else{  
                cerr << ">> ERROR!! No RigidBody specified for measured_cp" << endl;
                return -1;
            }
        }
    }

    if (node[interface->m_name]["setpoint_cp"]){
        for (size_t i = 0; i < node[interface->m_name]["setpoint_cp"].size(); i++){
            if(node[interface->m_name]["setpoint_cp"][i]["rigidbody"]){
                cerr << "[INFO!] Adding setpoint_cp ... " << endl;
                if (node[interface->m_name]["setpoint_cp"][i]["reference"]){
                    string objectName = node[interface->m_name]["setpoint_cp"][i]["reference"].as<string>();
                    interface->m_referenceSetpointPtr = (afBaseObjectPtr)m_worldPtr->getRigidBody(objectName);
                }

                string rigidName = node[interface->m_name]["setpoint_cp"][i]["rigidbody"].as<string>();
                afRigidBodyPtr rigidBodyPtr = m_worldPtr->getRigidBody(rigidName);

                if(!rigidBodyPtr){
                    cerr << ">> ERROR!! No RigidBody Named " << node[interface->m_name]["setpoint_cp"][i]["rigidbody"].as<string>() << " for setpoint_cp" << endl;
                    return -1;
                }
                
                if(node[interface->m_name]["setpoint_cp"][i]["namespace"]){
                    nspace = node[interface->m_name]["setpoint_cp"][i]["namespace"].as<string>();
                    interface->m_measuredCPRBsPtr[nspace] = rigidBodyPtr;

                    if (interface->m_referenceSetpointPtr){
                        interface->crtkInterface->add_setpoint_cp(nspace);
                    }
                    interface->crtkInterface->add_setpoint_cp(nspace + "/local/");
                }
                else{
                    rigidName = getNamefromPtr((afBaseObjectPtr)rigidBodyPtr);

                    if (interface->m_referenceSetpointPtr){
                        nspace = interface->m_name + '/' + rigidName;
                        interface->m_setpointCPRBsPtr[nspace] = rigidBodyPtr;
                        interface->crtkInterface->add_setpoint_cp(nspace); 
                    }
                    nspace = interface->m_name + "/local/" + rigidName;
                    interface->m_setpointCPRBsPtr[nspace] = rigidBodyPtr;
                    interface->crtkInterface->add_setpoint_cp(nspace); 
                }
            }

            else{  
                cerr << ">> ERROR!! No RigidBody specified for setpoint_cp" << endl;
                return -1;
            }
        }
        
    }
    
    if (node[interface->m_name]["measured_js"]){
        vector<string> jointNames;
        cerr << "[INFO!] Adding measured_js ... " << endl;
        vector<afJointPtr> jointsPtr;

        for (size_t i = 0; i < node[interface->m_name]["measured_js"].size(); i++){
            jointNames.clear();
            jointsPtr.clear();

            for (size_t j = 0; j < node[interface->m_name]["measured_js"][i]["joints"].size(); j++){
                string jointName = node[interface->m_name]["measured_js"][i]["joints"][j].as<string>();
                jointNames.push_back(jointName);
                jointsPtr.push_back(m_worldPtr->getJoint(jointName));
            }

            if(node[interface->m_name]["measured_js"][i]["namespace"]){
                nspace = node[interface->m_name]["measured_js"][i]["namespace"].as<string>();
            }
            else{
                nspace = interface->m_name;
            }
            interface->m_measuredJointsPtr[nspace] = (jointsPtr);
            interface->crtkInterface->add_measured_js(nspace, jointNames);
        }
    }

    if (node[interface->m_name]["measured_cf"]){
        if(node[interface->m_name]["measured_cf"]["rigidbody"]){
            cerr << "[INFO!] Adding measured_cf ... " << endl;
            for (int j = 0; j < node[interface->m_name]["measured_cf"].size(); j++){
                string rigidName = node[interface->m_name]["measured_cf"][j]["rigidbody"].as<string>();
                afRigidBodyPtr rigidBodyPtr = m_worldPtr->getRigidBody(rigidName);                
                
                if(!rigidBodyPtr){
                    cerr << ">> ERROR!! No RigidBody Named " << node[interface->m_name]["measured_cf"][j]["rigidbody"].as<string>() << " for measured_cf" << endl;
                    return -1;
                }

                if(node[interface->m_name]["measured_cf"]["namespace"]){
                    nspace = node[interface->m_name]["measured_cf"][j]["namespace"].as<string>();
                }
                else{
                    rigidName = getNamefromPtr((afBaseObjectPtr)rigidBodyPtr);
                    nspace = interface->m_name + "/" + rigidName;                
                }
                interface->m_measuredCFRBsPtr[nspace] = (rigidBodyPtr);
                interface->crtkInterface->add_measured_cf(nspace);    
            }
        }

        else{  
            cerr << ">> ERROR!! No RigidBody specified for measured_cf" << endl;
            return -1;
        }
    }

    if (node[interface->m_name]["servo_cp"]){
        for (size_t i = 0; i < node[interface->m_name]["servo_cp"].size(); i++){
            if(node[interface->m_name]["servo_cp"][i]["rigidbody"]){
                cerr << "[INFO!] Adding servo_cp ... " << endl;
                if (node[interface->m_name]["servo_cp"][i]["reference"]){
                    string objectName = node[interface->m_name]["servo_cp"][i]["reference"].as<string>();
                    interface->m_referenceServoPtr =  (afBaseObjectPtr)m_worldPtr->getRigidBody(objectName);
                }
                
                string rigidName = node[interface->m_name]["servo_cp"][i]["rigidbody"].as<string>();
                afRigidBodyPtr rigidBodyPtr = m_worldPtr->getRigidBody(rigidName);
                if(!rigidBodyPtr){
                    cerr << ">> ERROR!! No RigidBody Named " << node[interface->m_name]["servo_cp"][i]["rigidbody"].as<string>() << " for servo_cp" << endl;
                    return -1;
                }

                if(node[interface->m_name]["servo_cp"][i]["namespace"]){
                    nspace = node[interface->m_name]["servo_cp"][i]["namespace"].as<string>();
                    interface->m_servoCPRBsPtr[nspace] = rigidBodyPtr;

                    if (interface->m_referenceServoPtr){
                        interface->crtkInterface->add_servo_cp(nspace);
                    }
                    interface->m_servoCPRBsPtr[nspace + "/local/"] = rigidBodyPtr;
                    interface->crtkInterface->add_servo_cp(nspace + "/local/");
                }
                else{
                    rigidName = getNamefromPtr((afBaseObjectPtr)rigidBodyPtr);
                    if (interface->m_referenceServoPtr){
                        nspace = interface->m_name + "/" + rigidName;
                        interface->m_servoCPRBsPtr[nspace] = rigidBodyPtr;
                        interface->crtkInterface->add_servo_cp(nspace);
                    }
                    nspace = interface->m_name + "/local/" +  rigidName;
                    interface->m_servoCPRBsPtr[nspace] = rigidBodyPtr;
                    interface->crtkInterface->add_servo_cp(nspace);
                }
            }

            else{  
                cerr << ">> ERROR!! No RigidBody specified for servo_cp" << endl;
                return -1;
            }
        }

    }

    if (node[interface->m_name]["servo_jp"]){
        cerr << "[INFO!] Adding servo_jp ... " << endl;
        vector<string> jointNames;
        vector<afJointPtr> jointsPtr;
        for (size_t i = 0; i < node[interface->m_name]["servo_jp"].size(); i++){
            for (size_t j = 0; j < node[interface->m_name]["servo_jp"][i]["joints"].size(); j++){
                string jointName = node[interface->m_name]["servo_jp"][i]["joints"][j].as<string>();
                jointNames.push_back(jointName);
                jointsPtr.push_back(m_worldPtr->getJoint(jointName));
            }

            if(node[interface->m_name]["servo_jp"][i]["namespace"]){
                nspace = node[interface->m_name]["servo_jp"]["namespace"].as<string>();
            }
            else{
                nspace = interface->m_name;
            }
            interface->m_servoJointsPtr[nspace] = jointsPtr;
            interface->crtkInterface->add_servo_jp(nspace);
        }        
    }

    if (node[interface->m_name]["servo_cf"]){
        if(node[interface->m_name]["servo_cf"]["rigidbody"]){
            cerr << "[INFO!] Adding servo_cf ... " << endl;
            for (int j = 0; j < node[interface->m_name]["servo_cf"].size(); j++){
                string rigidName = node[interface->m_name]["servo_cf"][j]["rigidbody"].as<string>();
                afRigidBodyPtr rigidBodyPtr = m_worldPtr->getRigidBody(rigidName);
                if(!rigidBodyPtr){
                    cerr << ">> ERROR!! No RigidBody Named " << node[interface->m_name]["servo_cf"][j]["rigidbody"].as<string>() << " for servo_cf" << endl;
                    return -1;
                }
                
                if(node[interface->m_name]["servo_cf"]["namespace"]){
                    nspace = node[interface->m_name]["servo_cf"][j]["namespace"].as<string>();
                }
                else{
                    rigidName = getNamefromPtr((afBaseObjectPtr)rigidBodyPtr);
                    nspace = interface->m_name + "/" + rigidName;
                }
                interface->m_servoCFRBsPtr[nspace] = rigidBodyPtr;
                interface->crtkInterface->add_servo_cf(nspace);
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


void afCRTKBasePlugin::runOperatingState(Interface* interface){
    interface->crtkInterface->run_operating_state();
}

void afCRTKBasePlugin::runStateCommand(Interface* interface){
    interface->crtkInterface->run_state_command();
}

void afCRTKBasePlugin::runMeasuredCP(Interface* interface){
    // measured_cp
    cTransform referenceMeasuredCP, referenceToMeasuredCP;
    if (interface->m_measuredCPRBsPtr.size() > 0){
        for (const auto& pairNamePtr : interface->m_measuredCPRBsPtr) {
            // PairPtr: <string, Ptr>: namespace, rigidPtr
            cTransform measured_cp = pairNamePtr.second->getLocalTransform();
            // Use find() to locate the substring
            size_t foundLocal = pairNamePtr.first.find("local");

            // cerr << pairNamePtr.first << endl;

            if (foundLocal == std::string::npos && interface->m_referenceMeasuredPtr){
                referenceMeasuredCP = interface->m_referenceMeasuredPtr->getLocalTransform();
                referenceMeasuredCP.invert();
                referenceToMeasuredCP = referenceMeasuredCP * measured_cp;
                interface->crtkInterface->measured_cp(referenceToMeasuredCP, pairNamePtr.first);
            }
            else if (foundLocal != std::string::npos){
                interface->crtkInterface->measured_cp(measured_cp, pairNamePtr.first);  
            }
        }
    }

    // measured_cp
    if (interface->m_measuredObjectPtr.size() > 0){
        for (const auto& pairNamePtr : interface->m_measuredCPRBsPtr) {
            // PairPtr: <string, Ptr>: namespace, rigidPtr
            cTransform measured_cp = pairNamePtr.second->getLocalTransform();
            size_t foundLocal = pairNamePtr.first.find("local");

            if (foundLocal == std::string::npos && interface->m_referenceMeasuredPtr){
                referenceMeasuredCP = interface->m_referenceMeasuredPtr->getLocalTransform();
                referenceMeasuredCP.invert();
                referenceToMeasuredCP = referenceMeasuredCP * measured_cp;
                interface->crtkInterface->measured_cp(referenceToMeasuredCP, pairNamePtr.first);
            }

            else if (foundLocal != std::string::npos){
                interface->crtkInterface->measured_cp(measured_cp, pairNamePtr.first);  
            }
        }
    }
}

void afCRTKBasePlugin::runSetpointCP(Interface* interface){
    // setpoint_cp for RigidBody
    cTransform referenceSetpointCP, referenceToSetpointCP;
    if (interface->m_setpointCPRBsPtr.size() > 0){
        for (const auto& pairNamePtr : interface->m_setpointCPRBsPtr) {
            // PairPtr: <string, Ptr>: namespace, rigidPtr
            cTransform setpoint_cp = pairNamePtr.second->getLocalTransform();
            size_t foundLocal = pairNamePtr.first.find("local");

            if (foundLocal == std::string::npos && interface->m_referenceSetpointPtr){
                referenceSetpointCP = interface->m_referenceSetpointPtr->getLocalTransform();
                referenceSetpointCP.invert();
                referenceToSetpointCP = referenceSetpointCP * setpoint_cp;
                interface->crtkInterface->setpoint_cp(referenceToSetpointCP, pairNamePtr.first);
            }
            else if (foundLocal != std::string::npos){
                interface->crtkInterface->setpoint_cp(setpoint_cp, pairNamePtr.first);  
            }
        }
    }

    // Setpoint for object
    if (interface->m_setpointCPRBsPtr.size() > 0){
        for (const auto& pairNamePtr : interface->m_setpointCPRBsPtr) {
            // PairPtr: <string, Ptr>: namespace, rigidPtr
            cTransform setpoint_cp = pairNamePtr.second->getLocalTransform();
            size_t foundLocal = pairNamePtr.first.find("local");
            if (foundLocal == std::string::npos && interface->m_referenceSetpointPtr){
                referenceSetpointCP = interface->m_referenceSetpointPtr->getLocalTransform();
                referenceSetpointCP.invert();
                referenceToSetpointCP = referenceSetpointCP * setpoint_cp;
                interface->crtkInterface->setpoint_cp(referenceToSetpointCP, pairNamePtr.first);
            }
            else if (foundLocal != std::string::npos){
                interface->crtkInterface->setpoint_cp(setpoint_cp, pairNamePtr.first);  
            }
        }
    }
}


void afCRTKBasePlugin::runMeasuredJS(Interface* interface){
    // measured_js
    vector<string> jointName;
    if (interface->m_measuredJointsPtr.size() > 0){
        for (const auto& pairNamePtr : interface->m_measuredJointsPtr){
            jointName.clear();
            vector<double> measured_js;
            for  (size_t i = 0; i < pairNamePtr.second.size(); i++){
                double jointPos = pairNamePtr.second[i]->getPosition();
                measured_js.push_back(jointPos);
                jointName.push_back(pairNamePtr.second[i]->getName());
            }
            interface->crtkInterface->measured_js(measured_js, jointName, pairNamePtr.first);
        }
    }
}


void afCRTKBasePlugin::runMeasuredCF(Interface* interface){
    // measured_cf
    if (interface->m_measuredCFRBsPtr.size() > 0){
        for (const auto& pairNamePtr : interface->m_measuredCFRBsPtr) {
            btVector3 bt_measured_cf = pairNamePtr.second->m_estimatedForce;
            vector<double> measured_cf{bt_measured_cf.getX(),bt_measured_cf.getY(),bt_measured_cf.getZ(),0,0,0};
            interface->crtkInterface->measured_cf(measured_cf);
        }
    }
}



void afCRTKBasePlugin::runServoCP(Interface* interface, double dt){
    // servo_cp
    if (interface->m_servoCPRBsPtr.size() > 0){
        cTransform servo_cp;
        for (const auto& pairNamePtr : interface->m_servoCPRBsPtr) {
            if(interface->crtkInterface->servo_cp(servo_cp)){
                // Move the rigid body
                btTransform Tcommand = to_btTransform(servo_cp);

                // If the rigidbody is static
                if (pairNamePtr.second->m_bulletRigidBody->isStaticOrKinematicObject()){
                    pairNamePtr.second->m_bulletRigidBody->getMotionState()->setWorldTransform(Tcommand);
                    pairNamePtr.second->m_bulletRigidBody->setWorldTransform(Tcommand);
                }
                
                // If the rigid body is non-static
                else{
                    // Get current location     
                    btTransform curr_trans = pairNamePtr.second->getCOMTransform();

                    btVector3 pCommand, rCommand;
                    // Use the internal Cartesian Position Controller to Compute Output
                    pCommand = pairNamePtr.second->m_controller.computeOutput<btVector3>(curr_trans.getOrigin(), Tcommand.getOrigin(), dt);
                    // Use the internal Cartesian Rotation Controller to Compute Output
                    rCommand = pairNamePtr.second->m_controller.computeOutput<btVector3>(curr_trans.getBasis(), Tcommand.getBasis(), dt);
                    
                    // Set controller param here if needed
                    if (pairNamePtr.second->m_controller.m_positionOutputType == afControlType::FORCE){
                        pairNamePtr.second->m_bulletRigidBody->applyCentralForce(pCommand);
                        pairNamePtr.second->m_bulletRigidBody->applyTorque(rCommand);
                    }

                    else if (pairNamePtr.second->m_controller.m_positionOutputType == afControlType::VELOCITY){
                        pairNamePtr.second->m_bulletRigidBody->setLinearVelocity(pCommand);
                        pairNamePtr.second->m_bulletRigidBody->setAngularVelocity(rCommand);
                    }
                }   
            }   
        }
    }
    // servo_cp for baseObject
    if (interface->m_servoObjectPtr.size() > 0){
        cTransform servo_cp;
        for (const auto& pairNamePtr : interface->m_servoObjectPtr) {
            if(interface->crtkInterface->servo_cp(servo_cp)){
                pairNamePtr.second->setLocalTransform(servo_cp);
            }   
        }
    }
}


void afCRTKBasePlugin::runServoJP(Interface* interface){
    // servo_jp
    if (interface->m_servoJointsPtr.size() > 0){
        vector<double> servo_jp;
        if(interface->crtkInterface->servo_jp(servo_jp)){

            for (const auto& pairNamePtr : interface->m_servoJointsPtr) {
                for  (size_t i = 0; i < pairNamePtr.second.size(); i++){
                    if(pairNamePtr.second[i]){
                        pairNamePtr.second[i]->commandPosition(servo_jp[i]);
                    }
                }
            }
        }
    }
}


void afCRTKBasePlugin::runServoCF(Interface* interface){
    // servo_cf
    if (interface->m_servoCFRBsPtr.size() > 0){
        for (const auto& pairNamePtr : interface->m_servoCFRBsPtr) {
            vector<double> servo_cf;
            if(interface->crtkInterface->servo_cf(servo_cf)){
                pairNamePtr.second->applyForce(cVector3d(servo_cf[0], servo_cf[1], servo_cf[2]));
                pairNamePtr.second->applyTorque(cVector3d(servo_cf[3], servo_cf[4], servo_cf[5]));
            }
        }
    }
}


string afCRTKBasePlugin::getNamefromPtr(afBaseObjectPtr baseBodyPtr){
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