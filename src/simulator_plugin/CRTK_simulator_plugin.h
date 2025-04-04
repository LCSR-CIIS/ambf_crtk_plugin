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


// To silence warnings on MacOS
#define GL_SILENCE_DEPRECATION
#include <afFramework.h>
#include <yaml-cpp/yaml.h>
#include <regex>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>

#include "../utils/afCRTKInterface.h"
#include "../utils/CRTK_base_plugin.h"

namespace boost{
    namespace program_options{
        class variables_map;
    }
}

namespace p_opt = boost::program_options;

using namespace std;
using namespace ambf;


class afCRTKSimulatorPlugin: public afSimulatorPlugin, public afCRTKBasePlugin{
    public:
        afCRTKSimulatorPlugin();
        virtual int init(int argc, char** argv, const afWorldPtr a_afWorld) override;
        virtual void keyboardUpdate(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods) override;
        virtual void graphicsUpdate() override;
        virtual void physicsUpdate(double dt) override;
        virtual void reset() override;
        virtual bool close() override;

    protected:
        int loadCRTKInterfaceFromSimulator();
        string m_current_filepath;
        map<string, Interface*> m_namespaces;
        
};


AF_REGISTER_SIMULATOR_PLUGIN(afCRTKSimulatorPlugin)
