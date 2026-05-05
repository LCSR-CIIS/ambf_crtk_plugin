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
    runMeasuredCP(m_interface[0]);
    runServoCP(m_interface[0], dt);
}


int afCRTKObjectPlugin::loadCRTKInterfacefromObject(){
    string ns = m_objectAttribs->m_identificationAttribs.m_namespace;
    ns.pop_back();  // remove trailing '/'
    ns = ns.substr(ns.find_last_of('/') + 1);    
    m_interface.push_back(new Interface(ns));

    string objectName = m_objectPtr->getName(); // BODY name_of_rigidBody

    m_interface[0]->crtkInterface->add_measured_cp(ns + "/" + objectName);
    m_interface[0]->crtkInterface->add_servo_cp(ns + "/" + objectName);
    m_interface[0]->m_measuredObjectPtr[objectName] = m_objectPtr; 
    m_interface[0]->m_servoObjectPtr[objectName] = m_objectPtr; 

    return 1;
}


void afCRTKObjectPlugin::reset(){
}


bool afCRTKObjectPlugin::close(){
    return true;
}