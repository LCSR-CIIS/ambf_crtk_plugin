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


// To silence warnings on MacOS
#define GL_SILENCE_DEPRECATION
#include <afFramework.h>
#include <boost/algorithm/string.hpp>
#include <regex>

#include "../utils/afCRTKInterface.h"
#include "../utils/CRTK_base_plugin.h"

using namespace std;
using namespace ambf;

class afCRTKModelPlugin: public afModelPlugin, public afCRTKBasePlugin{
    public:
        afCRTKModelPlugin();
        virtual int init(const afModelPtr a_modelPtr, afModelAttribsPtr a_modelAttribs) override;
        virtual void graphicsUpdate() override;
        virtual void physicsUpdate(double dt) override;
        virtual void reset() override;
        virtual bool close() override;

    protected:
        int loadCRTKInterfacefromModel();
    // private:
        // Pointer to the world
        afModelPtr m_modelPtr;
        string m_configPath;

        bool m_isInitialized = false;

        map<string, Interface*> m_namespaces;
};


AF_REGISTER_MODEL_PLUGIN(afCRTKModelPlugin)
