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


// To silence warnings on MacOS
#define GL_SILENCE_DEPRECATION
#include <afFramework.h>
#include <yaml-cpp/yaml.h>

#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include "afCRTKInterface.h"
#include <regex>

#include "afConversions.h"

namespace boost{
    namespace program_options{
        class variables_map;
    }
}

namespace p_opt = boost::program_options;

using namespace std;
using namespace ambf;

class Interface{
    public:
        Interface(string ifname);
        string m_name;
        afType m_type;
        afCRTKInterface* crtkInterface;

        // AMBF Pointer
        // Joint related Pointers
        vector<afJointPtr> m_measuredJointsPtr, m_servoJointsPtr;
        vector<afJointPtr> m_measuredCPJointsPtr, m_servoCPJointsPtr;
        // RigidBody Pointers
        vector<afRigidBodyPtr> m_measuredCPRBsPtr, m_measuredCFRBsPtr, m_servoCPRBsPtr, m_servoCFRBsPtr;
        vector<afRigidBodyPtr> m_setpointCPRBsPtr;
        
        // Non RigidBody Pointers
        vector<afBaseObjectPtr> m_measuredObjectPtr, m_servoObjectPtr, m_measuredReferencePtr, m_servoReferencePtr;
        vector<afBaseObjectPtr> m_setpointObjectPtr;
        // Pointer for reference
        afBaseObjectPtr m_referenceMeasuredPtr = nullptr, m_referenceSetpointPtr = nullptr, m_referenceServoPtr = nullptr;
};

class afCRTKBasePlugin{
    public:
        afCRTKBasePlugin();

    protected:
        int readConfigFile(string config_filepath);
        int InitInterface(YAML::Node& node, Interface* interface);
        void runOperatingState(Interface* interface);   
        void runMeasuredCP(Interface* interface);
        void runSetpointCP(Interface* interface);
        void runMeasuredJS(Interface* interface);
        void runMeasuredCF(Interface* interface);
        void runServoCP(Interface* interface, double dt);
        void runServoJP(Interface* interface);
        void runServoCF(Interface* interface);

        string getNamefromPtr(afBaseObjectPtr baseBodyPtr);

        // Pointer to the world
        afWorldPtr m_worldPtr;

        int m_numInterface;
        vector<Interface*> m_interface;
};

