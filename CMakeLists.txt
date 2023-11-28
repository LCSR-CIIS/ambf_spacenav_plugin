#  Software License Agreement (BSD License)
#  Copyright (c) 2019-2023, AMBF.
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
#  $Author: Adnan Munawar $

cmake_minimum_required (VERSION 3.1)
project (ambf_spacenav_plugin)

set(CMAKE_CXX_STANDARD 11)

find_package(AMBF)
find_package(Boost COMPONENTS program_options filesystem)

include_directories(${AMBF_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})


link_directories(${AMBF_LIBRARY_DIRS})
add_definitions(${AMBF_DEFINITIONS})

# For spacenav control plugin
add_library(spacenav_plugin SHARED
    src/camera_panel_manager.cpp
    src/camera_panel_manager.h
    src/spacenav_control_plugin.cpp
    src/spacenav_control_plugin.h
    src/spacenav_manager.cpp
    src/spacenav_manager.h
    src/volume_manager.cpp
    src/volume_manager.h
    src/ros_interface.cpp
    src/ros_interface.h
    )

target_link_libraries (spacenav_plugin ${Boost_LIBRARIES} ${AMBF_LIBRARIES} spnav)
set_property(TARGET spacenav_plugin PROPERTY POSITION_INDEPENDENT_CODE TRUE)

if(NOT BUILD_PLUGIN_WITH_ROS)
    set(BUILD_PLUGIN_WITH_ROS TRUE)
endif()

if(BUILD_PLUGIN_WITH_ROS)
    add_definitions(-DBUILD_WITH_ROS)
endif()
