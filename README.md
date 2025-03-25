# AMBF CRTK Plugin
This project is a plugin for Asynchronous Multibody Framework ([AMBF](https://github.com/WPI-AIM/ambf)) developed by Munawar et al. 
This plugin will enable users to communicate with AMBF simulation using [Collaborative Robotics Toolkit(CRTK)](https://github.com/collaborative-robotics).

Please visit [here](https://github.com/collaborative-robotics/documentation) for the CRTK docmuentation.


## 1. Installation Instructions:
Lets call the absolute location of this package as **<plugin_path>**. E.g. if you cloned this repo in your home folder, **<plugin_path>** = `~/ambf_crtk_plugin/` OR `/home/<username>/ambf_crtk_plugin`.

### 1.1 clone and build 
```bash
cd <catkin_ws or ros2_ws>
cd src
git clone git@github.com:LCSR-CIIS/ambf_crtk_plugin.git
cd <catkin_ws or ros2_ws>
catkin build or colcon_build
```

### 1.3 Source crtk_msgs
Follow the instruction in [crtk_msgs](https://github.com/collaborative-robotics/crtk_msgs) and don't forget to source it.
```bash
source catkin_ws/devel/setup.bash # ROS1
source ros2_ws/install/setup.bash # ROS2
```

## 2. How to use your plugin
You can test this plugin on the example by:
`<ambf_exe_dir> ---> e.g. ~/ambf/bin/lin-x86_64`

### 2.1 Simulator plugin
You are required to specify configuration file such as `example/CRTK_config.yaml`:
```bash
cd <ambf_exe_dir>
./ambf_simulator --plugins <plugin_path>/build/libambf_crtk_simulator_plugin.so --conf <plugin_path>/example/CRTK_config.yaml
```
You can also define plugin in your `launch.yaml`: 

```bash
plugins: [
  {
    name: CRTK,
    filename: libambf_crtk_simulator_plugin.so,
    path: <plugin_path>/ambf_crtk_simulator_plugin/build
  }
]
```

### 2.2 Model plguin
You can specify plugin in your ADF file as follows:
```bash
plugins: [
  {
    name: CRTK,
    filename: libambf_crtk_model_plugin.so,
    path: <plugin_path>/ambf_crtk_model_plugin/build
  }
]
crtk_config: <path_to_your_configuration_file> # relative to your ambf_crtk_model_plugin/src/model_plugin
```

### 2.3 Object plugin
You can specify plugin in your ADF file as follows:
```bash
plugins: [
  {
    name: CRTK,
    filename: libambf_crtk_object_plugin.so,
    path: <plugin_path>/ambf_crtk_object_plugin/build
  }
]
```

## 3. Configuration file
You are required to specify your custom made configuration file to specify what kind of objects you want to monitor/control with which CRTK command.
```CRTK_config.yaml

# Interfaces used for this plugin
interface:
- REMS/Research
- Robot
- Atracsys

# For Interface "REMS/Research"
REMS/Research:
  measured_cp:
    # Additonal namespace
    # rostopic name will be "REMS/Research/delta/measured_cp" 
    namespace: delta        # <- This option is optional

    # Name of Rigidbody in AMBF 
    rigidbody: Endoscope Tip

  measured_js:
    # Name of joints in AMBF
    joints:
    - carriage3_joint
    - carriage1_joint
    - carriage2_joint
    - roll_joint


  measured_cf:
    # Name of Rigidbody in AMBF 
    rigidbody: Endoscope Tip

Robot:
  servo_cp:
    # Name of Rigidbody in AMBF 
    rigidbody: Endoscope Tip
  
  servo_jp:
    # Name of joints in AMBF 
    joints:
    - carriage3_joint
    - carriage1_joint
    - carriage2_joint
    - roll_joint

  servo_cf:
    # Additonal namespace
    # rostopic name will be "Robot/compliance/servo_cf" 
    namespace: compliance # <- This option is optional
    
    # Name of Rigidbody in AMBF 
    rigidbody: Endoscope Tip

Atracsys:
  servo_cp:
    # Name of Rigidbody in AMBF 
    rigidbody: Endoscope Tip
```

In this example, there will be the following rostopics:
```
/REMS/Research/delta/measured_cp
/REMS/Research/measured_js
/REMS/Research/measured_cf
/Robot/servo_cp
/Robot/servo_jp
/Robot/compliance/servo_cf
/Atracsys/servo_cp
```

## Example command 
Please refer to [Surgical Robotics Challenge](https://github.com/surgical-robotics-ai/surgical_robotics_challenge) and use the following command to use it for SRC:

```bash
ambf_simulator --launch_file ~/surgical_robotics_challenge/launch.yaml -l 0,1,2,3,4,5 --plugins ./build/libambf_crtk_simulator_plugin.so --conf example/SRC_config.yaml 
```

You can use the following example:
```bash
ambf_simulator -a ~/3D-Slicer_ROS_Module_with_AMBF/AMBF_Plugin_3DSlicer/ADF/galen.yaml --plugins ./build/libambf_crtk_simulator_plugin.so --conf example/CRTK_config.yaml 
```

You can use the following example to use model plugin for dvrk:
```bash
ambf_simulator --launch_file launch.yaml -l 0,1,2
```

## Trouble Shooting
Please refer to [AMBF helper](https://github.com/LCSR-CIIS/AMBF_helper) for installation procedure and how to debug the plugins.