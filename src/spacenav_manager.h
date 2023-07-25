//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019-2023, AMBF
    (https://github.com/WPI-AIM/ambf)

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
*/
//==============================================================================

#ifndef SPACENAV_MANAGER_H
#define SPACENAV_MANAGER_H

#include "ros/ros.h"
#include <afFramework.h>

#include <sensor_msgs/Joy.h>
#include <spnav.h>

using namespace chai3d;
using namespace ambf;
using namespace std;


class SpaceNavControl{

    public:
        SpaceNavControl();

        int init(afWorldPtr a_afWorld, afCameraPtr &a_camera);
        int measured_jp();
        void controlCamera();
        void controlObject(afBaseObjectPtr objectPtr);
        void controlRigidBody(afRigidBodyPtr rigidBodyPtr);
        void controlCObject(cShapeSphere* objectPtr);
        void close();

    // private:

        // Pointer to the world/camera
        afWorldPtr m_worldPtr;
        afCameraPtr m_camera;
        afVolumePtr m_volume;
        afRigidBodyPtr m_rigidBody;

        // Spacenav related param
        vector<double> m_scale; 
        vector<double> m_deadbound; // if the value is less than this value, then we regard them as "static"
        int m_staticCountThres;
        int m_noMotion = 0;

        cVector3d m_trans;
        cVector3d m_rot;

        vector<double> m_buttons;
        bool m_spanavEnable = false;
        spnav_event sev;

        int count = 0;

        double m_scale_linear;
        double m_scale_angular;


};



#endif //SPACENAV_MANAGER_H