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
    // Add operating state
    interface->crtkInterface->add_operating_state("");
    
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
                interface->m_measuredCPRBsPtr.push_back(rigidBodyPtr);
                rigidName = getNamefromPtr((afBaseObjectPtr)rigidBodyPtr);
                
                if(node[interface->m_name]["measured_cp"][i]["namespace"]){
                    if (interface->m_referenceMeasuredPtr){
                        interface->crtkInterface->add_measured_cp(node[interface->m_name]["measured_cp"][i]["namespace"].as<string>() + rigidName);
                    }
                    interface->crtkInterface->add_measured_cp(node[interface->m_name]["measured_cp"][i]["namespace"].as<string>() + "/local/" + rigidName);
                }
                else{
                    if (interface->m_referenceMeasuredPtr){
                        interface->crtkInterface->add_measured_cp(rigidName); 
                    }
                    interface->crtkInterface->add_measured_cp("local/" +rigidName); 
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
                interface->m_setpointCPRBsPtr.push_back(rigidBodyPtr);
                rigidName = getNamefromPtr((afBaseObjectPtr)rigidBodyPtr);
                
                if(node[interface->m_name]["setpoint_cp"][i]["namespace"]){
                    if (interface->m_referenceSetpointPtr){
                        interface->crtkInterface->add_setpoint_cp(node[interface->m_name]["setpoint_cp"][i]["namespace"].as<string>() + rigidName);
                    }
                    interface->crtkInterface->add_setpoint_cp(node[interface->m_name]["setpoint_cp"][i]["namespace"].as<string>() + "/local/" + rigidName);
                }
                else{
                    if (interface->m_referenceSetpointPtr){
                        interface->crtkInterface->add_setpoint_cp(rigidName); 
                    }
                    interface->crtkInterface->add_setpoint_cp("local/" +rigidName); 
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
        for (size_t j = 0; j < node[interface->m_name]["measured_js"]["joints"].size(); j++){
            string jointName = node[interface->m_name]["measured_js"]["joints"][j].as<string>();
            interface->m_measuredJointsPtr.push_back(m_worldPtr->getJoint(jointName));
            jointNames.push_back(jointName);
        }

        if(node[interface->m_name]["measured_js"]["namespace"])
            interface->crtkInterface->add_measured_js(node[interface->m_name]["measured_js"]["namespace"].as<string>(), jointNames);
        else
            interface->crtkInterface->add_measured_js("", jointNames);
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
                interface->m_measuredCFRBsPtr.push_back(rigidBodyPtr);
                rigidName = getNamefromPtr((afBaseObjectPtr)rigidBodyPtr);

                if(node[interface->m_name]["measured_cf"]["namespace"]){
                    interface->crtkInterface->add_measured_cf(node[interface->m_name]["measured_cf"][j]["namespace"].as<string>()+ '/' + rigidName);
                }
                else{
                    interface->crtkInterface->add_measured_cf(rigidName);                    
                }
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

                interface->m_servoCPRBsPtr.push_back(rigidBodyPtr);
            
                rigidName = getNamefromPtr((afBaseObjectPtr)rigidBodyPtr);

                if(node[interface->m_name]["servo_cp"][i]["namespace"]){
                    if (interface->m_referenceServoPtr){
                        interface->crtkInterface->add_servo_cp(node[interface->m_name]["servo_cp"][i]["rigidbody"]["namespace"].as<string>() + "/" + rigidName);
                    }
                    interface->crtkInterface->add_servo_cp(node[interface->m_name]["servo_cp"][i]["rigidbody"]["namespace"].as<string>() + "/local/" + rigidName);
                }
                else{
                    if (interface->m_referenceServoPtr){
                        interface->crtkInterface->add_servo_cp(rigidName);
                    }
                    interface->crtkInterface->add_servo_cp("local/" + rigidName);
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
        for (size_t j = 0; j < node[interface->m_name]["servo_jp"]["joints"].size(); j++){
            string jointName = node[interface->m_name]["servo_jp"]["joints"][j].as<string>();
            jointNames.push_back(jointName);
            interface->m_servoJointsPtr.push_back(m_worldPtr->getJoint(jointName));
        }

        if(node[interface->m_name]["servo_jp"]["namespace"])
            interface->crtkInterface->add_servo_jp(node[interface->m_name]["servo_jp"]["namespace"].as<string>());
        else
            interface->crtkInterface->add_servo_jp("");
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
                interface->m_servoCFRBsPtr.push_back(rigidBodyPtr);
                
                rigidName = getNamefromPtr((afBaseObjectPtr)rigidBodyPtr);
                if(node[interface->m_name]["servo_cf"]["namespace"])
                    interface->crtkInterface->add_servo_cf(node[interface->m_name]["servo_cf"][j]["namespace"].as<string>() + '/' + rigidName);
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


void afCRTKBasePlugin::runOperatingState(Interface* interface){
    interface->crtkInterface->run_operating_state();
}

void afCRTKBasePlugin::runMeasuredCP(Interface* interface){
    // measured_cp
    cTransform referenceMeasuredCP, referenceToMeasuredCP;
    if (interface->m_measuredCPRBsPtr.size() > 0){
        for (size_t i = 0; i < interface->m_measuredCPRBsPtr.size(); i++){
            cTransform measured_cp = interface->m_measuredCPRBsPtr[i]->getLocalTransform();
            if (interface->m_referenceMeasuredPtr){
                referenceMeasuredCP = interface->m_referenceMeasuredPtr->getLocalTransform();
                referenceMeasuredCP.invert();
                referenceToMeasuredCP = referenceMeasuredCP * measured_cp;
                interface->crtkInterface->measured_cp(referenceToMeasuredCP, getNamefromPtr((afBaseObjectPtr)interface->m_measuredCPRBsPtr[i]));
                
            }
            interface->crtkInterface->measured_cp(measured_cp, "local/" + getNamefromPtr((afBaseObjectPtr)interface->m_measuredCPRBsPtr[i]));
        }
    }

    // measured_cp
    if (interface->m_measuredObjectPtr.size() > 0){
        for (size_t i = 0; i < interface->m_measuredObjectPtr.size(); i++){
            cTransform measured_cp = interface->m_measuredObjectPtr[i]->getLocalTransform();
            if (interface->m_referenceMeasuredPtr){
                referenceMeasuredCP = interface->m_referenceMeasuredPtr->getLocalTransform();
                referenceMeasuredCP.invert();
                referenceToMeasuredCP = referenceMeasuredCP * measured_cp;
                interface->crtkInterface->measured_cp(referenceToMeasuredCP, getNamefromPtr((afBaseObjectPtr)interface->m_measuredObjectPtr[i]));
            }
            interface->crtkInterface->measured_cp(measured_cp, "local/" + getNamefromPtr((afBaseObjectPtr)interface->m_measuredObjectPtr[i]));
        }
    }
}

void afCRTKBasePlugin::runSetpointCP(Interface* interface){
    // measured_cp
    cTransform referenceSetpointCP, referenceToSetpointCP;
    if (interface->m_setpointCPRBsPtr.size() > 0){
        for (size_t i = 0; i < interface->m_setpointCPRBsPtr.size(); i++){
            cTransform setpoint_cp = interface->m_setpointCPRBsPtr[i]->getLocalTransform();
            if (interface->m_referenceSetpointPtr){
                referenceSetpointCP = interface->m_referenceSetpointPtr->getLocalTransform();
                referenceSetpointCP.invert();
                referenceToSetpointCP = referenceSetpointCP * setpoint_cp;
                interface->crtkInterface->setpoint_cp(referenceToSetpointCP, getNamefromPtr((afBaseObjectPtr)interface->m_setpointCPRBsPtr[i]));
                
            }
            interface->crtkInterface->setpoint_cp(setpoint_cp, "local/" + getNamefromPtr((afBaseObjectPtr)interface->m_setpointCPRBsPtr[i]));
        }
    }

    // measured_cp
    if (interface->m_setpointObjectPtr.size() > 0){
        for (size_t i = 0; i < interface->m_measuredObjectPtr.size(); i++){
            cTransform measured_cp = interface->m_measuredObjectPtr[i]->getLocalTransform();
            if (interface->m_referenceMeasuredPtr){
                referenceSetpointCP = interface->m_referenceMeasuredPtr->getLocalTransform();
                referenceSetpointCP.invert();
                referenceToSetpointCP = referenceSetpointCP * measured_cp;
                interface->crtkInterface->setpoint_cp(referenceToSetpointCP, getNamefromPtr((afBaseObjectPtr)interface->m_setpointObjectPtr[i]));
            }
            interface->crtkInterface->setpoint_cp(measured_cp, "local/" + getNamefromPtr((afBaseObjectPtr)interface->m_setpointObjectPtr[i]));
        }
    }
}


void afCRTKBasePlugin::runMeasuredJS(Interface* interface){
    // measured_js
    if (interface->m_measuredJointsPtr.size() > 0){
        vector<double> measured_js;
        for  (size_t i = 0; i < interface->m_measuredJointsPtr.size(); i++){
            double jointPos = interface->m_measuredJointsPtr[i]->getPosition();
            measured_js.push_back(jointPos);
        }
        interface->crtkInterface->measured_js(measured_js);
    }
}


void afCRTKBasePlugin::runMeasuredCF(Interface* interface){
    // measured_cf
    if (interface->m_measuredCFRBsPtr.size() > 0){
        for (size_t i = 0; i < interface->m_measuredCFRBsPtr.size(); i++){
            btVector3 bt_measured_cf = interface->m_measuredCFRBsPtr[i]->m_estimatedForce;
            vector<double> measured_cf{bt_measured_cf.getX(),bt_measured_cf.getY(),bt_measured_cf.getZ(),0,0,0};
            interface->crtkInterface->measured_cf(measured_cf);
        }
    }
}


void afCRTKBasePlugin::runServoCP(Interface* interface, double dt){
    // servo_cp
    if (interface->m_servoCPRBsPtr.size() > 0){
        cTransform servo_cp;
        for (size_t i = 0; i < interface->m_servoCPRBsPtr.size(); i++){
            if(interface->crtkInterface->servo_cp(servo_cp)){
                // Move the rigid body
                btTransform Tcommand = to_btTransform(servo_cp);

                // If the rigidbody is static
                if (interface->m_servoCPRBsPtr[i]->m_bulletRigidBody->isStaticOrKinematicObject()){
                    interface->m_servoCPRBsPtr[i]->m_bulletRigidBody->getMotionState()->setWorldTransform(Tcommand);
                    interface->m_servoCPRBsPtr[i]->m_bulletRigidBody->setWorldTransform(Tcommand);
                }
                
                // If the rigid body is non-static
                else{
                    // Get current location     
                    btTransform curr_trans = interface->m_servoCPRBsPtr[i]->getCOMTransform();

                    btVector3 pCommand, rCommand;
                    // Use the internal Cartesian Position Controller to Compute Output
                    pCommand = interface->m_servoCPRBsPtr[i]->m_controller.computeOutput<btVector3>(curr_trans.getOrigin(), Tcommand.getOrigin(), dt);
                    // Use the internal Cartesian Rotation Controller to Compute Output
                    rCommand = interface->m_servoCPRBsPtr[i]->m_controller.computeOutput<btVector3>(curr_trans.getBasis(), Tcommand.getBasis(), dt);
                    
                    // Set controller param here if needed
                    if (interface->m_servoCPRBsPtr[i]->m_controller.m_positionOutputType == afControlType::FORCE){
                        interface->m_servoCPRBsPtr[i]->m_bulletRigidBody->applyCentralForce(pCommand);
                        interface->m_servoCPRBsPtr[i]->m_bulletRigidBody->applyTorque(rCommand);
                    }

                    else if (interface->m_servoCPRBsPtr[i]->m_controller.m_positionOutputType == afControlType::VELOCITY){
                        interface->m_servoCPRBsPtr[i]->m_bulletRigidBody->setLinearVelocity(pCommand);
                        interface->m_servoCPRBsPtr[i]->m_bulletRigidBody->setAngularVelocity(rCommand);
                    }
                }   
            }   
        }
    }
    // servo_cp  for baseObject
    if (interface->m_servoObjectPtr.size() > 0){
        cTransform servo_cp;
        for (size_t i = 0; i < interface->m_servoObjectPtr.size(); i++){
            if(interface->crtkInterface->servo_cp(servo_cp)){
                interface->m_servoObjectPtr[i]->setLocalTransform(servo_cp);
            }   
        }
    }
}


void afCRTKBasePlugin::runServoJP(Interface* interface){
    // servo_jp
    if (interface->m_servoJointsPtr.size() > 0){
        vector<double> servo_jp;
        if(interface->crtkInterface->servo_jp(servo_jp)){
            for  (size_t i = 0; i < interface->m_servoJointsPtr.size(); i++){
                if(interface->m_servoJointsPtr[i]){
                    interface->m_servoJointsPtr[i]->commandPosition(servo_jp[i]);
                }
            }
        }
    }
}


void afCRTKBasePlugin::runServoCF(Interface* interface){
    // servo_cf
    if (interface->m_servoCFRBsPtr.size() > 0){
        for (size_t i = 0; i < interface->m_servoCFRBsPtr.size(); i++){
            vector<double> servo_cf;
            if(interface->crtkInterface->servo_cf(servo_cf)){
                interface->m_servoCFRBsPtr[i]->applyForce(cVector3d(servo_cf[0], servo_cf[1], servo_cf[2]));
                interface->m_servoCFRBsPtr[i]->applyTorque(cVector3d(servo_cf[3], servo_cf[4], servo_cf[5]));
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