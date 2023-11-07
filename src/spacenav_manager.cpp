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

#include "spacenav_manager.h"

using namespace std;

SpaceNavControl::SpaceNavControl(){

}

// Rotate the camera
int SpaceNavControl::init(afWorldPtr a_afWorld, afCameraPtr &a_camera)
{   
    cerr << "> Initializing SpaceNav..." << endl;
    m_worldPtr = a_afWorld;

    // Define the camera
    m_camera = a_camera;
   
    // Set the scaling 
    // TODO: Hardcoded now 
    double full_scale = 0.001 / 512.0;
    m_scale = {0.5 * full_scale, 0.5 * full_scale, 0.5 * full_scale, 50.0 * full_scale, 50.0 *full_scale, 50.0 *full_scale};

    double trans_db = 0.1;
    double rot_db = 0.1;
    m_deadbound = {trans_db, trans_db, trans_db, rot_db, rot_db, rot_db};

    // The number of polls needed to be done before the device is considered "static"
    m_staticCountThres = 100;

    // Scaling used for moving the rigid body
    m_scale_linear = 100.0;
    m_scale_angular = 2.0;

    // Initialize the value
    m_trans.set(0,0,0);
    m_rot.set(0,0,0);
    m_buttons = {0.0, 0.0};

    int result = spnav_open();

    cerr << "SpaceNav Condition: " << result << endl;
    if (result == -1) // 0: successfully open, -1: failed
    {
        cerr << "Could not open the space navigator device. " << endl;
        m_spanavEnable = false;
        return -1;
    }
    m_spanavEnable = true;

    return 1;
}

int SpaceNavControl::measured_jp()
{   
    if (m_spanavEnable){
        switch (spnav_poll_event(&sev))
        {
        case 0:
            if (++m_noMotion > m_staticCountThres){

                if (fabs(m_trans.x()) < m_deadbound[0] && \
                fabs(m_trans.y()) < m_deadbound[1] && \
                fabs(m_trans.z()) < m_deadbound[2])
                {
                    m_trans.set(0.0, 0.0, 0.0);
                }

                if (fabs(m_rot.x()) < m_deadbound[3] && \
                fabs(m_rot.y()) < m_deadbound[4] && \
                fabs(m_rot.z()) < m_deadbound[5])
                {
                    m_rot.set(0.0, 0.0, 0.0);
                } 
            }
            m_noMotion = 0;
            break;

        case SPNAV_EVENT_MOTION:

            if (abs(sev.motion.z < 510) && abs(sev.motion.x < 510) && abs(sev.motion.y < 510) && abs(sev.motion.rz < 510 && abs(sev.motion.rx < 510)) && abs(sev.motion.ry < 510)){
                m_trans.set(-sev.motion.z  * m_scale[0], sev.motion.x * m_scale[1], sev.motion.y  * m_scale[2]);
                m_rot.set(sev.motion.rz * m_scale[3], -sev.motion.rx * m_scale[4], sev.motion.ry* m_scale[5]);
            }

            break;
        
        case SPNAV_EVENT_BUTTON:
            m_buttons[sev.button.bnum]++;

            break;
            
        default:
            cerr << "Unknown message type in spacenav. This should never happen." << endl;
            return -1;
            break;
        }
        return 1;
    }

    return -1;
}

// Control Camera
void SpaceNavControl::controlCamera(afCameraPtr cameraPtr)
{   
    int result = measured_jp();
    cMatrix3d rotation;
    if (result == 1){ 
        cameraPtr->setLocalPos(cameraPtr->getLocalPos() + cameraPtr->getLocalRot() * m_trans);
        rotation.setExtrinsicEulerRotationDeg(m_rot.x(), m_rot.y(), m_rot.z(), C_EULER_ORDER_ZYX);
        cameraPtr->setLocalRot(cameraPtr->getLocalRot() * rotation);
    }
}

// Control Object
void SpaceNavControl::controlObject(afBaseObjectPtr objectPtr){
    int result = measured_jp();
    cMatrix3d rotation;
    if (objectPtr && result == 1){
        objectPtr->setLocalPos(objectPtr->getLocalPos() + m_camera->getLocalRot() * m_trans);
        
        rotation.setExtrinsicEulerRotationDeg(-m_scale_angular * m_rot.x(), -m_scale_angular * m_rot.y(), m_scale_angular * m_rot.z(), C_EULER_ORDER_XYZ);
        cMatrix3d cam_rot = m_camera->getLocalRot();
        cam_rot.invert();
        cMatrix3d rotation1 = cam_rot * rotation * m_camera->getLocalRot();
        objectPtr->setLocalRot(rotation1 * objectPtr->getLocalRot());
    }

}

// Control RigidBody Object
void SpaceNavControl::controlRigidBody(afRigidBodyPtr rigidBodyPtr){
    int result = measured_jp();
    cMatrix3d rotation;
    if (rigidBodyPtr && result == 1){
        btVector3 trans;
        trans.setValue((m_camera->getLocalRot() * m_trans).x() * m_scale_linear,\
         (m_camera->getLocalRot() * m_trans).y() * m_scale_linear,\
          (m_camera->getLocalRot() * m_trans).z()* m_scale_linear);

        rigidBodyPtr->m_bulletRigidBody->setLinearVelocity(trans);

        btVector3 rot;
        rot.setValue(-m_rot.x(), -m_rot.y(), m_rot.z());
        rigidBodyPtr->m_bulletRigidBody->setAngularVelocity(rot);
    }
}

// Control Object
void SpaceNavControl::controlCObject(cShapeSphere* objectPtr){
    int result = measured_jp();
    cMatrix3d rotation;
    if (objectPtr){
        objectPtr->setLocalPos(objectPtr->getLocalPos() + m_camera->getLocalRot() * m_trans);
        rotation.setExtrinsicEulerRotationDeg(-m_rot.x(), -m_rot.y(), m_rot.z(), C_EULER_ORDER_XYZ);
        cMatrix3d cam_rot = m_camera->getLocalRot();
        cam_rot.invert();
        cMatrix3d rotation1 = cam_rot * rotation * m_camera->getLocalRot();
        objectPtr->setLocalRot(rotation1 * objectPtr->getLocalRot());
    }

}

void SpaceNavControl::close(){
    spnav_close();
}