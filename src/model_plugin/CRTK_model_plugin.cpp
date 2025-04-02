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

#include "CRTK_model_plugin.h"

afCRTKModelPlugin::afCRTKModelPlugin(){
    cout << "/*********************************************" << endl;
    cout << "/* AMBF Model Plugin for CRTK Interface" << endl;
    cout << "/*********************************************" << endl;
}


int afCRTKModelPlugin::init(const afModelPtr a_modelPtr, afModelAttribsPtr a_modelAttribs){
    // Store Pointer for the world
    m_modelPtr = a_modelPtr;
    m_worldPtr = m_modelPtr->getWorldPtr();     //Ptr to the simulation world

    // Improve the constratint <- This will make the model unstable
    // m_worldPtr->m_bulletWorld->getSolverInfo().m_erp = 1.0;  
    // m_worldPtr->m_bulletWorld->getSolverInfo().m_erp2 = 1.0; 

    // Define path
    string file_path = __FILE__;
    string current_path = file_path.substr(0, file_path.rfind("/"));
    YAML::Node specificationDataNode = YAML::Load(a_modelAttribs->getSpecificationData().m_rawData);

    // If there is a configuration file given in the ADF file
    if (specificationDataNode["plugins"][0]["crtk_config"]){
        m_configPath = current_path + "/" + specificationDataNode["plugins"][0]["crtk_config"].as<string>();
        cerr << "[INFO!] Reading configuration file: " << m_configPath << endl;
        // return readConfigFile(m_configPath);
        return 1;
    }
    
    // If there is no configuration file given
    else{
        cerr << "WARNING! NO configuration file specified." << endl;
        int result = loadCRTKInterfacefromModel();
        return 1;
    }
}


void afCRTKModelPlugin::graphicsUpdate(){
    if (!m_isInitialized){
        int result = readConfigFile(m_configPath);
        if (result == 1){
            m_isInitialized = true;
        }
    }
}

void afCRTKModelPlugin::physicsUpdate(double dt){
    if (m_isInitialized){
        for (Interface* interface:m_interface){
            runOperatingState(interface);
            runMeasuredCP(interface);
            runSetpointCP(interface);
            runMeasuredJS(interface);
            runMeasuredCF(interface);
            runServoCP(interface, dt);
            runServoJP(interface);
            runServoCF(interface);
        }
    }
}

int afCRTKModelPlugin::loadCRTKInterfacefromModel(){
        //ChildrenMap: map<map<afType, map<string, afBaseObject*> > > 
    afChildrenMap::iterator cIt;
    afChildrenMap* childrenMap = m_modelPtr->getChildrenMap();
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

    return 1;
}

void afCRTKModelPlugin::reset(){
}


bool afCRTKModelPlugin::close(){
    return true;
}