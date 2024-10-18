# AMBF SpaceNav plugin

This project is a plugin for Asynchronous Multibody Framework ([AMBF](https://github.com/WPI-AIM/ambf)) developed by Munawar et al. 
This plugin will enable users to  control objects in AMBF simulation using 3Dconnexion SpaceNavigator 6DOF joystick.

![SpaceNav_fig](/figs/spacenav.png)

## 1. Installation Instructions:
Lets call the absolute location of this package as **<plugin_path>**. E.g. if you cloned this repo in your home folder, **<plugin_path>** = `~/ambf_spacenav_plugin/` OR `/home/<username>/ambf_spacenav_plugin`.

### 1.1 Install driver for spacenav
```
sudo apt install spacenavd
```


### 1.2 clone and build 
```bash
git clone git@github.com:LCSR-CIIS/ambf_spacenav_plugin.git
cd ambf_spacenav_plugin
mkdir build && cd build
cmake .. -DBUILD_PLUGIN_WITH_ROS=False
make
```

A simple package.xml has been included in this repository to enable catkin to find and build it **<plugin_path>** should be located within `catkin_ws/src/`.
```bash
cd <catkin_ws>
catkin build ambf_spacenav_plugin
```

## 2. How to use your plugin
You can test this plugin on the example by:
`<ambf_exe_dir> ---> e.g. ~/ambf/bin/lin-x86_64`

Without the configuration file.
```bash
cd <ambf_exe_dir>
./ambf_simulator --plugins <plugin_path>/build/lib_spacenav_plugin.so
```

With configuration file such as `example/spacenav_config.yaml`:
```bash
cd <ambf_exe_dir>
./ambf_simulator --plugins <plugin_path>/build/lib_spacenav_plugin.so --spf <plugin_path>/example/spacenav_config.yaml
```

## 3. Configuration file
You can specify your custom made configuration file to specify what kind of objects you want to control.
```spacenav_config.yaml

# Controlable objects
control objects:
- CAMERA main_camera
- VOLUME mastoidectomy_volume
- BODY drill_tip
- BODY drill_reference
- BODY Base
```

And other parameters regarding the scaling and the speed for the spacenav.
```
# Scaling for the motion
scaling: [0.000001 , 0.000001, 0.000001, 0.0001, 0.0001, 0.0001]

# Deadbound
deadbound:
  translation: 0.1
  rotation: 0.1


# The number of polls needed to be done 
# before the device is considered "static"
static count threshold: 100

# Scaling used for moving the rigid bodys
velocity sclaing:
  linear: 100.0
  angular: 2.0
```

### 3.1 Slicing Volume
If you add the following line in your configuration you will be able to slice the volume in the scene.

```spacenav_config.yaml
# Define if you need want to slice volume or not
slice volume:
  matcap path: /home/hishida3/adnan/volumetric_drilling/resources/matcap/00ShinyWhite.jpg
```
While you are selecting the VOLUME, press the right button on youy spcaenav to activate "sliciing mode".


## 4. Keyboard shorcuts
`[Ctrl + L]` : show/hide list of controllable objects.


### 5. Rotation Frame
Currently, the camera frame will rotate around its own frame and other objects will move according to the camera frame.

You can modify the Rotation axis or frame by changing the `setExtrinsicEulerRotationDeg` in "src/spacenav_manager.cpp."