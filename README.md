# AMBF CRTK Plugin
This project is a plugin for Asynchronous Multibody Framework ([AMBF](https://github.com/WPI-AIM/ambf)) developed by Munawar et al. 
This plugin will enable users to communicate with AMBF simulation using [Collaborative Robotics Toolkit(CRTK)](https://github.com/collaborative-robotics).

Please visit [here](https://github.com/collaborative-robotics/documentation) for the CRTK docmuentation.


## 1. Installation Instructions:
Lets call the absolute location of this package as **<plugin_path>**. E.g. if you cloned this repo in your home folder, **<plugin_path>** = `~/ambf_crtk_plugin/` OR `/home/<username>/ambf_crtk_plugin`.

### 1.1 clone and build 
```bash
git clone git@github.com:LCSR-CIIS/ambf_crtk_plugin.git
cd ambf_crtk_plugin
mkdir build && cd 
make
```

## 2. How to use your plugin
You can test this plugin on the example by:
`<ambf_exe_dir> ---> e.g. ~/ambf/bin/lin-x86_64`

You are required to specify configuration file such as `example/CRTK_config.yaml`:
```bash
cd <ambf_exe_dir>
./ambf_simulator --plugins <plugin_path>/build/libambf_crtk_plugin.so --conf <plugin_path>/example/CRTK_config.yaml
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

## Trouble Shooting

