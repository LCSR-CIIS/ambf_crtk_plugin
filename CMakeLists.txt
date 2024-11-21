#  Software License Agreement (BSD License)
#  Copyright (c) 2019-2024, AMBF.
#  (https://github.com/WPI-AIM/ambf)
#
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#  * Redistributions of source code must retain the above copyright
#  notice, this list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above
#  copyright notice, this list of conditions and the following
#  disclaimer in the documentation and/or other materials provided
#  with the distribution.
#
#  * Neither the name of authors nor the names of its contributors may
#  be used to endorse or promote products derived from this software
#  without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#  $Author: Hisashi Ishida $

cmake_minimum_required (VERSION 3.1)
project (ambf_crtk_plugin)

set(CMAKE_CXX_STANDARD 11)

find_package(AMBF)
find_package(Boost COMPONENTS program_options filesystem)

include_directories(${AMBF_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})


link_directories(${AMBF_LIBRARY_DIRS})
add_definitions(${AMBF_DEFINITIONS})

# For Simulator Plugin
add_library(ambf_crtk_simulator_plugin SHARED
    src/utils/afCRTKInterface.cpp
    src/utils/afCRTKInterface.h
    src/utils/CRTK_base_plugin.cpp
    src/utils/CRTK_base_plugin.h
    src/simulator_plugin/CRTK_simulator_plugin.cpp
    src/simulator_plugin/CRTK_simulator_plugin.h
    )

target_link_libraries (ambf_crtk_simulator_plugin ${Boost_LIBRARIES} ${AMBF_LIBRARIES})
set_property(TARGET ambf_crtk_simulator_plugin PROPERTY POSITION_INDEPENDENT_CODE TRUE)


# Model Plugin
add_library(ambf_crtk_model_plugin SHARED
    src/utils/afCRTKInterface.cpp
    src/utils/afCRTKInterface.h
    src/utils/CRTK_base_plugin.cpp
    src/utils/CRTK_base_plugin.h
    src/model_plugin/CRTK_model_plugin.cpp
    src/model_plugin/CRTK_model_plugin.h
    )
target_link_libraries (ambf_crtk_model_plugin ${Boost_LIBRARIES} ${AMBF_LIBRARIES})
set_property(TARGET ambf_crtk_model_plugin PROPERTY POSITION_INDEPENDENT_CODE TRUE)

# Object Plugins
add_library(ambf_crtk_object_plugin SHARED
    src/utils/afCRTKInterface.cpp
    src/utils/afCRTKInterface.h
    src/utils/CRTK_base_plugin.cpp
    src/utils/CRTK_base_plugin.h   
    src/object_plugin/CRTK_object_plugin.cpp
    src/object_plugin/CRTK_object_plugin.h
    )

target_link_libraries (ambf_crtk_object_plugin ${Boost_LIBRARIES} ${AMBF_LIBRARIES})
set_property(TARGET ambf_crtk_object_plugin PROPERTY POSITION_INDEPENDENT_CODE TRUE)

if(NOT BUILD_PLUGIN_WITH_ROS)
    set(BUILD_PLUGIN_WITH_ROS TRUE)
endif()

if(BUILD_PLUGIN_WITH_ROS)
    add_definitions(-DBUILD_WITH_ROS)
endif()