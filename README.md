# AMBF CRTK Plugin
[Caution] `main/ros2` branch is for ROS2, if you are using ROS1, use `ros1` branch instead.
This project is a plugin for Asynchronous Multibody Framework ([AMBF](https://github.com/WPI-AIM/ambf)) developed by Munawar et al. 
This plugin will enable users to communicate with AMBF simulation using [Collaborative Robotics Toolkit(CRTK)](https://github.com/collaborative-robotics).

Please visit [here](https://github.com/collaborative-robotics/documentation) for the CRTK docmuentation.


## 1. Installation Instructions:
Lets call the absolute location of this package as **<plugin_path>**. E.g. if you cloned this repo in your home folder, **<plugin_path>** = `~/ambf_crtk_plugin/` OR `/home/<username>/ambf_crtk_plugin`.

### 1.1 clone and build (For ROS2)
Follow the instruction for ROS2 installation. Make sure you do not source ros1 in your .bashrc file.
Current ROS2 implementation is only available as a Adnan's fork (`devel` branch). 
Make sure to use the correct repo and branch. 

[Caution!] This is currently tested only for ROS2.

```bash
mkdir ros2_ws # Create ros2_ws
cd ros2_ws
cd src
git clone git@github.com:adnanmunawar/ambf.git
cd ambf
git checkout devel # switch to devel branch
cd .. # go back to src 
git clone git@github.com:LCSR-CIIS/ambf_crtk_plugin.git
cd ambf_crtk_plugin
cd ..
```

### 1.3 Source crtk_msgs
Follow these instructions to get [crtk_msgs](https://github.com/collaborative-robotics/crtk_msgs) and don't forget to source it.
```bash
git clone https://github.com/collaborative-robotics/ros2_crtk_msgs crtk/crtk_msgs
git clone https://github.com/collaborative-robotics/ros2_crtk_python_client crtk/crtk_python_client
cd ../ # make sure to colcon build in the root ros2_ws

colcon build

source catkin_ws/devel/setup.bash # ROS1
source ros2_ws/install/setup.bash # ROS2
```

## 2. How to use your plugin
You can test this plugin on the example by:

For ROS1, you can set the alias "ambf_simulator" and save it in the bashrc. [Note] This is not needed for ROS1.
<!-- `<ambf_exe_dir> ---> e.g. ~/ambf/bin/lin-x86_64` -->
```bash
cd <ambf_exe_dir> # e.g. ros_ws/build/AMBF/bin,
# optional: To execute ambf_simulator without having to be in the directory, one can set an alias
alias ambf_simulator=~/ros2_ws/build/AMBF/bin/ambf_simulator
# Save and close the file and reload by either relaunching the terminal or typing 
. ~/.bashrc
```

For ROS2, once you source the following command you should be able to use the alias `ambf_simulator`,
```bash
source ros2_ws/install/setup.bash # ROS2
```

With the alias set, ambf_simulator can be executed from a terminal from any location


### 2.1 Simulator plugin
You are required to specify configuration file such as `example/CRTK_config.yaml`:

Assuming you are in <ros_ws>:
<plugin_path> is where the plugins build to, e.g. `./build/ambf_crtk_plugin`
<config_path> is the parent folder of the `_config.yaml` file, e.g. `./src/ambf_crtk_plugin/example/plugin-config/simulator_plugin`

```bash
ambf_simulator --plugins <plugin_path>/libambf_crtk_simulator_plugin.so --conf <config_path>/CRTK_config.yaml
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

### 2.2 Model plugin
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
```yaml
# AMBF CRTK configuration file template for CRTK_model_plugin
# Author: Hisashi Ishida
# Date: 04.04.2025


# Interfaces used for this plugin
interface:
- PSM1

PSM1:
  measured_cp: [
    {
      rigidbody: psm1/BODY yaw link,
      namespace: /PSM1/, # [Optional] Override namespace "/PSM1/measured_cp"
      reference: BODY tool link
    },
    # {
    #   rigidbody: psm1/BODY base link,
    #   reference: CameraFrame
    # }
  ]

  setpoint_cp: [
    {
      rigidbody: psm1/BODY yaw link,
      namespace: /PSM1/, # [Optional] Override namespace "/PSM1/measured_cp"
      reference: BODY tool link
    }
  ]

  measured_js: [
    {
      namespace: /PSM1,
      joints: [
        psm1/JOINT base link-yaw link,
        psm1/JOINT pitch bottom link-pitch end link,
        psm1/JOINT pitch end link-main insertion link,
        psm1/JOINT pitch front link-pitch bottom link,
        psm1/JOINT pitch top link-pitch end link,

        psm1/JOINT tool roll link-tool pitch link,
        psm1/JOINT yaw link-pitch back link,
        psm1/JOINT yaw link-pitch end link,
        psm1/JOINT yaw link-pitch front link,
      ]
    },
    {
      joints: [
        psm1/JOINT tool pitch link-tool gripper1 link,
        psm1/JOINT tool pitch link-tool gripper2 link,
      ],
      namespace: /PSM1/gripper
    }
  ]
    
  servo_cp:
  [
    {
      rigidbody: psm1/BODY yaw link,
      namespace: /PSM1/, # [Optional] Override namespace "/PSM1/servo_cp"
      reference: BODY tool link
    }
  ]

  servo_jp: [
    {
      namespace: /PSM1,
      joints: [
        psm1/JOINT base link-yaw link,
        psm1/JOINT main insertion link-tool roll link,
        psm1/JOINT pitch back link-pitch bottom link,
        psm1/JOINT pitch back link-pitch top link,
        psm1/JOINT pitch bottom link-pitch end link,

        psm1/JOINT tool roll link-tool pitch link,
        psm1/JOINT yaw link-pitch back link,
        psm1/JOINT yaw link-pitch end link,
        psm1/JOINT yaw link-pitch front link,
      ]
    },
    {
      joints: [
        psm1/JOINT tool pitch link-tool gripper1 link,
        psm1/JOINT tool pitch link-tool gripper2 link,
      ],
      namespace: /PSM1/gripper
    }
  ]
    
```

In this example, there will be the following rostopics:
```bash
/PSM1/gripper/measured_js
/PSM1/gripper/servo_jp
/PSM1/local/measured_cp  # This measured_cp is w.r.t ambf origin
/PSM1/local/servo_cp     # This servo_cp is w.r.t ambf origin
/PSM1/local/setpoint_cp  # This setpoint_cp is w.r.t ambf origin
/PSM1/measured_cp        # This measured_cp is w.r.t reference specified in the configuration file
/PSM1/measured_js
/PSM1/operating_state
/PSM1/servo_cp           # This servo_cp is w.r.t reference specified in the configuration file
/PSM1/servo_jp
/PSM1/setpoint_cp        # This setpoint_cp is w.r.t reference specified in the configuration file
/PSM1/state_command
```

## Example command 
Please refer to [Surgical Robotics Challenge](https://github.com/surgical-robotics-ai/surgical_robotics_challenge) and use the following command to use it for SRC:
 
```bash
ambf_simulator --launch_file ~/surgical_robotics_challenge/launch.yaml -l 0,1,2,3,4,5 --plugins <plugin_path>/libambf_crtk_simulator_plugin.so --conf <config_path>/SRC_config.yaml 
```

You can use the following example:
```bash
ambf_simulator -a ~/3D-Slicer_ROS_Module_with_AMBF/AMBF_Plugin_3DSlicer/ADF/galen.yaml --plugins <plugin_path>/libambf_crtk_simulator_plugin.so --conf <config_path>/CRTK_config.yaml 
```

You can use the following example to use model plugin for dvrk:
```bash
ambf_simulator --launch_file launch.yaml -l 0,1,2
```

You can use the following example to use model plugin for full dvrk blender model:
```bash
ambf_simulator --launch_file launch.yaml -l 3,4,5
```

## Trouble Shooting
Please refer to [AMBF helper](https://github.com/LCSR-CIIS/AMBF_helper) for installation procedure and how to debug the plugins.
