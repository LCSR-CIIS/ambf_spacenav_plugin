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

#include "spacenav_control_plugin.h"

using namespace std;

afSpaceNavControlPlugin::afSpaceNavControlPlugin(){
    cout << "/*********************************************" << endl;
    cout << "/* AMBF Plugin for SpaceNav Control" << endl;
    cout << "/*********************************************" << endl;
}

int afSpaceNavControlPlugin::init(int argc, char** argv, const afWorldPtr a_afWorld){
    p_opt::options_description cmd_opts("SpaceNav control Command Line Options");
    cmd_opts.add_options()
            ("info", "Show Info")
            ("spf", p_opt::value<string>()->default_value(""), "Name of specfile for spacenav control");

    p_opt::variables_map var_map;
    p_opt::store(p_opt::command_line_parser(argc, argv).options(cmd_opts).allow_unregistered().run(), var_map);
    p_opt::notify(var_map);

    if(var_map.count("info")){
        std::cout<< cmd_opts << std::endl;
        return -1;
    }

    // Loading options 
    string spec_filepath = var_map["spf"].as<string>();

    // Define path
    string file_path = __FILE__;
    m_current_filepath = file_path.substr(0, file_path.rfind("/"));

    // Initialize Camera
    m_worldPtr = a_afWorld;
    vector<string> cameraNames = {"main_camera", "cameraL", "cameraR", "stereoLR"};
    bool  initcam = initCamera(cameraNames);
    
    if (initcam){
        cerr << "SUCCESSFULLY Initialize Camera" << endl;
    }
    
    else{
        cerr << "ERROR! FAILED To Initialize Camera" << endl;
        return -1;
    }

    // Initialize SpaceNav 
    int result = m_spaceNavControl.init(m_worldPtr, m_cameras["main_camera"]);

    if (result == 1){
         cerr << "SUCCESSFULLY Initialized SpaceNav." <<  endl;
    }
    else{
        cerr << "INFO! Could not initilialize SpaceNav" << endl;
    }

    // When config file was defined
    if(!spec_filepath.empty()){
        cerr << "> loading the user defined specfile..." << endl; 
        cerr << spec_filepath << endl;
        if (loadConfigurationFile(spec_filepath) == -1){
            return -1;
        }
    }

    // No config file specified
    else{
        cerr << "No specfile loaded. Using the defualt config" << endl;
        loadControllableObjectsFromWorld();
    }

    // Initialize Labels
    bool initlabel = initLabels();
    m_num = m_controllableObjects.size();
    cout << "------------ Controlable Object Details ------------" << endl;
    cout << "# of Objects:" <<  m_num << endl;
    for (int i=0; i <  m_controllableObjects.size(); i++){
        cout << m_controllableObjects[i]->name_ << endl;
    }
    cout << "----------------------------------------------------" << endl;

    return 1;
}

bool afSpaceNavControlPlugin::initCamera(vector<string> cameraNames){
    cout << "> Initializing CAMERA ..." << endl;
    if (cameraNames.size() == 0){
        cerr << "ERROR! NO CAMERA Specified." << endl;
        return false;
    }
    

    for (int i = 0 ; i < cameraNames.size() ; i++){
        afCameraPtr cam = m_worldPtr->getCamera(cameraNames[i]);
        if (cam){
            cerr << "INFO! GOT CAMERA: " << cam->getName() << endl;
            m_cameras[cameraNames[i]] = cam;
        }
        // if there is no main_camera load the first camera from the world.
        if(cameraNames[i] == "main_camera" && !cam){
            cerr << "INFO! FAILED TO LOAD main_camera, taking the first camera from world " << endl;
            cam = m_worldPtr->getCameras()[0];
            m_cameras["main_camera"] = cam;
        }
    } 

    // Specify the camera for the text overlays
    m_panelManager.addCamera(m_cameras["main_camera"]);
    if (m_cameras["steroLR"]){
        m_cameras["steroLR"]->getInternalCamera()->m_stereoOffsetW = 0.1;
        m_panelManager.addCamera(m_cameras["main_camera"]);
    }

    cBackground* background = new cBackground();
    background->setCornerColors(cColorf(1.0f, 1.0f, 1.0f),
                                cColorf(1.0f, 1.0f, 1.0f),
                                cColorf(0.6f, 0.6f, 0.6f),
                                cColorf(0.6f, 0.6f, 0.6f));
    m_cameras["main_camera"]->getBackLayer()->addChild(background);

    return true;
}

bool afSpaceNavControlPlugin::changeCamera(afCameraPtr cameraPtr){
    cout << "> Changing CAMERA ..." << endl;

    // Specify the camera for the text overlays
    m_panelManager.addCamera(cameraPtr);
    cBackground* background = new cBackground();
    background->setCornerColors(cColorf(1.0f, 1.0f, 1.0f),
                                cColorf(1.0f, 1.0f, 1.0f),
                                cColorf(0.6f, 0.6f, 0.6f),
                                cColorf(0.6f, 0.6f, 0.6f));
    cameraPtr->getBackLayer()->addChild(background);

    return true;
}


bool afSpaceNavControlPlugin::initLabels(){
    // create a font
    cFontPtr font = NEW_CFONTCALIBRI20();

    // Active Object panel
    m_activeObjectLabel = new cLabel(font);
    m_activeObjectLabel->m_fontColor.setRedCrimson();
    m_activeObjectLabel->setCornerRadius(5, 5, 5, 5);
    m_activeObjectLabel->setShowPanel(true);
    m_activeObjectLabel->setColor(cColorf(1.0, 1.0, 1.0, 1.0));
    m_activeObjectLabel->setTransparencyLevel(0.6);
    m_activeObjectLabel->setText("Warning Panel Test for spaceNav contorl");

    m_panelManager.addPanel(m_activeObjectLabel, 0.01, 0.9, PanelReferenceOrigin::LOWER_LEFT, PanelReferenceType::NORMALIZED);
    m_panelManager.setVisible(m_activeObjectLabel, m_spaceNavControl.m_spanavEnable);

    // Objects List panel
    m_objectListLabel = new cLabel(font);
    m_objectListLabel->setFontScale(0.8);
    m_objectListLabel->m_fontColor.setBlack();
    m_objectListLabel->setCornerRadius(5, 5, 5, 5);
    m_objectListLabel->setShowPanel(true);
    m_objectListLabel->setColor(cColorf(1.0, 1.0, 1.0, 1.0));
    m_objectListLabel->setTransparencyLevel(0.6);
    m_objectListLabel->setText("List of Controlable objects");

    m_panelManager.addPanel(m_objectListLabel, 0.01, 0.2, PanelReferenceOrigin::LOWER_LEFT, PanelReferenceType::NORMALIZED);
    m_panelManager.setVisible(m_objectListLabel, m_spaceNavControl.m_spanavEnable);

    return true;
}

void afSpaceNavControlPlugin::keyboardUpdate(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods){ 
    if (a_mods == GLFW_MOD_CONTROL){
        if (a_key == GLFW_KEY_L) {
            if (m_spaceNavControl.m_spanavEnable){
                m_enableList = !m_enableList;
                m_panelManager.setVisible(m_objectListLabel, m_enableList);
            }
        }
    }
}

void afSpaceNavControlPlugin::graphicsUpdate(){
    if (m_activeContorlObject->publishState_ && m_isSendingInfo){
        m_panelManager.setText(m_activeObjectLabel, "Publishing state ...");
    }
    else if (m_activeContorlObject->sliceVolume_ && m_isSlicing){
        m_panelManager.setText(m_activeObjectLabel, "Slicing VOLUME" + m_activeContorlObject->name_);
    }
    else{
        m_panelManager.setText(m_activeObjectLabel, m_activeContorlObject->name_);
    }

    string list_text = "--- List of Controlable objects ---\n";
    for (int i = 0; i < m_num; i++){
        if (m_controllableObjects[i]->name_ == m_activeContorlObject->name_){
            list_text += "-> " + m_controllableObjects[i]->name_;
        }
        else{
            list_text += m_controllableObjects[i]->name_;
        }
        if (i < m_num){
            list_text +=  "\n";
        }
    }
    m_panelManager.setText(m_objectListLabel, list_text);
    m_panelManager.update();
}

void afSpaceNavControlPlugin::physicsUpdate(double dt)
{
    // Retrieve SpaceNav current status 
    m_spaceNavControl.measured_jp();

    // Get index from side buttons
    if (!m_useSingleButton){
        m_index = (int(m_spaceNavControl.m_buttons[0]/2) - int(m_spaceNavControl.m_buttons[1]/2)) % m_num;
    }
    else{
        m_index = (int(m_spaceNavControl.m_buttons[0]/2)) % m_num;
    }

    // Make sure the index is in the range of [0 - m_num]
    if (m_index < 0){
        m_index += m_num;
    }
    
    // Select the active control object
    m_activeContorlObject = m_controllableObjects[m_index];

    // If the slicing is functionality is activated
    if (m_activeContorlObject->sliceVolume_){
        if (int(m_spaceNavControl.m_buttons[1]/2) % 2 == 1){
            // Get the Max translation value and send the axis and value
            int axis = 0;
            double value = 0;
            m_spaceNavControl.getMaxTransValue(axis, value);
            m_voulmeManager.sliceVolume(axis, value);
            m_rosSlicingInterface.publishAxisValue(axis, value);

            m_isSlicing = true;
        }
        else {
            m_isSlicing = false;
        }
    }

    if (m_activeContorlObject->publishState_){
        if (int(m_spaceNavControl.m_buttons[1]/2) % 2 == 1){
                // Get the Max translation value and send the axis and value
                int axis = 0;
                double value = 0;
                m_spaceNavControl.getMaxTransValue(axis, value);
                m_rosInfoInterface.publishAxisValue(axis, value);

                m_isSendingInfo = true;
        }

        else {
            m_isSendingInfo = false;
        }
    }

    // If the object is CAMERA
    if(!m_isSendingInfo && m_activeContorlObject->objectPtr_->getType() == afType::CAMERA){
        if(m_activeContorlObject->name_ == "stereo_camera" && m_isStereo){
            for (int i = 0; i < m_stereoCameraPtr.size(); i++){
                m_spaceNavControl.controlCamera(m_stereoCameraPtr[i]);
            }
        }
        else{
            m_spaceNavControl.controlCamera(afCameraPtr(m_activeContorlObject->objectPtr_));
        }
    }

    // If the object is RIGIDBODY
    else if (!m_isSendingInfo && m_activeContorlObject->objectPtr_->getType() == afType::RIGID_BODY){
        m_spaceNavControl.controlRigidBody(afRigidBodyPtr(m_activeContorlObject->objectPtr_));
    }

    // If the object is VOLUME
    else if(!m_isSlicing && m_activeContorlObject->objectPtr_->getType() == afType::VOLUME){
        m_spaceNavControl.controlObject(m_activeContorlObject->objectPtr_);
    }

    // else{
    //     m_spaceNavControl.controlObject(m_activeControlObjectPtr);
    // }
}

int afSpaceNavControlPlugin::loadConfigurationFile(string spec_filepath){
    //Load the user defined object here. 
    YAML::Node node = YAML::LoadFile(spec_filepath);
    
    // Get contorl objects
    m_num = node["control objects"].size();

    bool isVolume = false;
    for (size_t i = 0; i < m_num; i++){   
        string objectName = node["control objects"][i].as<string>();
        cout << "Looking for the object: \"" << objectName << "\"" << endl;
        
        string objectType = objectName.substr(0, objectName.find(" "));
        objectName = objectName.substr(objectName.find(" "));
        objectName.erase(0, 1);

        // Get object type and the name
        afBaseObjectPtr objectPtr;
        if (objectType == "CAMERA"){
            objectPtr = m_worldPtr->getCamera(objectName);
        }
        else if (objectType == "LIGHT"){
            objectPtr = m_worldPtr->getLight(objectName);
        }
        else if (objectType == "VOLUME"){
            objectPtr = m_worldPtr->getVolume(objectName);
            m_volumeName = objectName;
            isVolume = true;
        }
        else if (objectType == "BODY"){
            objectPtr = m_worldPtr->getRigidBody(objectName);
        }
        else if (objectType == "JOINT"){
            objectPtr = m_worldPtr->getJoint(objectName);
        }
        else {
            objectPtr = m_worldPtr->getBaseObject(objectName, m_worldPtr->getChildrenMap());
        }
        
        // Check if the object was found or not
        if (objectPtr){
            cerr << "FOUND!!" << endl;
            ControllableObject* controllableObject = new ControllableObject;
            controllableObject->name_ = objectName;
            controllableObject->objectPtr_ = objectPtr;
            m_controllableObjects.push_back(controllableObject);
        }

        else {
            cerr << "ERROR! COULD NOT FIND OBJECT NAMED \"" << objectName << "\"" << endl;
            return -1;
        }    
    }

    // Get spacenav Parameters
    if (node["scaling"]){
        if (node["scaling"].as<std::vector<double>>().size() == 6){
            m_spaceNavControl.m_scale = node["scaling"].as<std::vector<double>>();
        }
        else{
            cerr << "ERROR in config file. The scaling has to be size 6." << endl;
        }
    }
    
    if (node["deadbound"]){
        if (node["deadbound"]["translation"]){
            double trans_db = node["deadbound"]["translation"].as<double>();
            m_spaceNavControl.m_deadbound[0] = trans_db;
            m_spaceNavControl.m_deadbound[1] = trans_db;
            m_spaceNavControl.m_deadbound[2] = trans_db;
        }
        if (node["deadbound"]["rotation"]){
            double rot_db = node["deadbound"]["rotation"].as<double>();
            m_spaceNavControl.m_deadbound[3] = rot_db;
            m_spaceNavControl.m_deadbound[4] = rot_db;
            m_spaceNavControl.m_deadbound[5] = rot_db;
        }
    }

    if (node["static count threshold"]){
        m_spaceNavControl.m_staticCountThres = node["static count threshold"].as<int>();
    }

    if (node["velocity scaling"]){
        if (node["velocity scaling"]["linear"]){
            m_spaceNavControl.m_scale_linear = node["velocity scaling"]["linear"].as<double>();
        }
        if (node["velocity scaling"]["angular"]){
            m_spaceNavControl.m_scale_angular = node["velocity scaling"]["angular"].as<double>();
        }
    }

    if(node["stereo_camera"]){
        m_isStereo = true;
        cout << "stereo" << endl;
        for (int i = 0; i < node["stereo_camera"].size(); i++){
            afCameraPtr cameraPtr = m_worldPtr->getCamera(node["stereo_camera"][i].as<string>());
            m_stereoCameraPtr.push_back(cameraPtr);
        }
        ControllableObject* controllableObject = new ControllableObject;
        controllableObject->name_ = "stereo_camera";
        controllableObject->objectPtr_ = m_stereoCameraPtr[0];
        m_controllableObjects.push_back(controllableObject);
    }

    if (node["slice volume"]){
        if (node["slice volume"] && isVolume){
            if (node["slice volume"]["volume name"]){
                // Check for the volume
                for (ControllableObject* object: m_controllableObjects){
                    cerr << object->name_ << endl;

                    // If the name exist and the type is VOLUME
                    if (object->name_ == node["slice volume"]["volume name"].as<string>() && object->objectPtr_->getType() == afType::VOLUME){
                        
                        object->sliceVolume_ = true;

                        // Check for matcap (Needed for volume slicing)
                        if (node["slice volume"]["matcap path"]){
                            bool result = m_voulmeManager.initVolume(m_worldPtr, m_volumeName, node["slice volume"]["matcap path"].as<string>());
                                if (!result){
                                    cerr << "[ERROR] CANNOT INITIALIZE VOLUME MANAGER!!" << endl;
                                    return -1;
                                }
                            m_rosSlicingInterface.init("/spacenav/VolumeSlicing/");
                            m_useSingleButton = true;
                            cerr << "Slicing Volume: " << object->sliceVolume_ << endl;  
                        }
                        else{
                            cerr << "[ERROR] NO MATCAP path defined!" << endl;
                        }
                    }
                }
            }
            else{
                cerr << "[ERROR] NO Volume named " << endl;
            }
        }
    }

    if (node["publish state"]){
        if (node["publish state"]["object"]){
            for (ControllableObject* object: m_controllableObjects){
                if (object->name_ == node["publish state"]["object"].as<string>()){
                    m_rosInfoInterface.init("/spacenav/State/");
                    object->publishState_ = true;
                    m_useSingleButton = true;
                }
            }
        }
    }

    return 1;
}

int afSpaceNavControlPlugin::loadControllableObjectsFromWorld(){
    // Load every Model/Object in the world
    // ModelMap: map<string, afModelPtr>
    afModelMap* map = m_worldPtr->getModelMap();
    for (auto it=map->begin(); it != map->end(); it++){
        //ChildrenMap: map<map<afType, map<string, afBaseObject*> >
        afChildrenMap::iterator cIt;
        afChildrenMap* childrenMap = it->second->getChildrenMap();

        for(cIt = childrenMap->begin(); cIt != childrenMap->end(); ++cIt){   
            for (auto it_child=cIt->second.begin(); it_child != cIt->second.end(); ++it_child){
                // Store the name and Ptr to the objects other than JOINT
                if (it_child->second->getType() != afType::JOINT){   
                    ControllableObject* controllableObject = new ControllableObject;
                    controllableObject->name_ = it_child->first;
                    controllableObject->objectPtr_ = it_child->second;
                    m_controllableObjects.push_back(controllableObject);
                }
            }
        }
    }
}

void afSpaceNavControlPlugin::reset(){
    cerr << "INFO! PLUGIN RESET CALLED" << endl;
}

bool afSpaceNavControlPlugin::close(){
    delete m_activeObjectLabel;
    delete m_objectListLabel;
    m_spaceNavControl.close();
    return -1;
}
