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
    \date      03.21.2024
    
*/
//==============================================================================

#include "CRTK_object_plugin.h"

afCRTKObjectPlugin::afCRTKObjectPlugin(){
    cout << "/*********************************************" << endl;
    cout << "/* AMBF Object Plugin for CRTK Interface" << endl;
    cout << "/*********************************************" << endl;
}


int afCRTKObjectPlugin::init(const afBaseObjectPtr a_afObjectPtr, const afBaseObjectAttribsPtr a_objectAttribs){

    // Store Pointer for the world
    m_objectPtr = a_afObjectPtr;
    m_objectAttribs = a_objectAttribs;
    
    loadCRTKInterfacefromObject();
    cerr << "INFO! Initialization Successfully Finished!!" << endl;
    return 1;
}


void afCRTKObjectPlugin::graphicsUpdate(){
}


void afCRTKObjectPlugin::physicsUpdate(double dt){
    // measured_cp
    if (m_interface[0]->m_measuredObjectPtr.size() > 0){
        for (size_t i = 0; i < m_interface[0]->m_measuredObjectPtr.size(); i++){
            cTransform measured_cp = m_interface[0]->m_measuredObjectPtr[i]->getLocalTransform();
            m_interface[0]->crtkInterface->measured_cp(measured_cp, getNamefromPtr((afBaseObjectPtr)m_interface[0]->m_measuredObjectPtr[i]));
        }
    }

     // servo_cp
    if (m_interface[0]->m_servoObjectPtr.size() > 0){
        cTransform servo_cp;
        for (size_t i = 0; i < m_interface[0]->m_servoObjectPtr.size(); i++){
            if(m_interface[0]->crtkInterface->servo_cp(servo_cp)){
                m_interface[0]->m_servoObjectPtr[i]->setLocalTransform(servo_cp);
            }   
        }
    }
}


int afCRTKObjectPlugin::loadCRTKInterfacefromObject(){
    string ns = m_objectAttribs->m_identificationAttribs.m_namespace;
    // cerr << ns.erase(0,9) << endl; // Erase "/ambf/env"
    ns = ns.erase(0,10);
    
    string objectName = m_objectAttribs->m_identifier; // BODY name_of_rigidBody
    vector<string> v;
    boost::split(v, objectName, boost::is_any_of(" ")); 
    objectName.erase(0,v[0].length()+1); //Remove BODY
    objectName = objectName; //ns + rigidName
    objectName = regex_replace(objectName, regex{" "}, string{"_"});

    m_interface.push_back(new Interface(ns));

    objectName = regex_replace(objectName, regex{" "}, string{"_"});
    m_interface[0]->crtkInterface->add_measured_cp(objectName);
    m_interface[0]->crtkInterface->add_servo_cp(objectName);
    m_interface[0]->m_measuredObjectPtr.push_back(m_objectPtr); 
    m_interface[0]->m_servoObjectPtr.push_back(m_objectPtr); 
    

    // This part is not working
    // No type defined in the m_identificationAttribs.m_objectType, since it is a baseObjectFrame
    // if (m_objectAttribs->m_identificationAttribs.m_objectType == afType::RIGID_BODY){
    //     cerr << "Object Type: RIGIDBODY" << endl;
    //     objectName = regex_replace(objectName, regex{" "}, string{"_"});
    //     m_interface[0]->crtkInterface->add_measured_cp(objectName);
    //     m_interface[0]->crtkInterface->add_servo_cp(objectName);
    //     m_interface[0]->m_measuredObjectPtr.push_back(m_objectPtr); 
    //     m_interface[0]->m_servoObjectPtr.push_back(m_objectPtr); 
    // }

    // if (m_objectAttribs->m_identificationAttribs.m_objectType == afType::LIGHT){
    //     cerr << "Object Type: LIGHT" << endl;
    //     objectName = regex_replace(objectName, regex{" "}, string{"_"});
    //     m_interface[0]->m_measuredObjectPtr.push_back(m_objectPtr);
    //     m_interface[0]->m_servoObjectPtr.push_back(m_objectPtr);
    //     m_interface[0]->crtkInterface->add_measured_cp(objectName);
    //     m_interface[0]->crtkInterface->add_servo_cp(objectName);
    // }

    // if (m_objectAttribs->m_identificationAttribs.m_objectType == afType::CAMERA){
    //     cerr << "Object Type: CAMERA" << endl;
    //     objectName = regex_replace(objectName, regex{" "}, string{"_"});
    //     m_interface[0]->m_measuredObjectPtr.push_back(m_objectPtr);
    //     m_interface[0]->m_servoObjectPtr.push_back(m_objectPtr);
    //     m_interface[0]->crtkInterface->add_measured_cp(objectName);
    //     m_interface[0]->crtkInterface->add_servo_cp(objectName);
    // }

    return 1;
}


void afCRTKObjectPlugin::reset(){
}


bool afCRTKObjectPlugin::close(){
    return true;
}