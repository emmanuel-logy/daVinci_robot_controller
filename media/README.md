# RBE501_final_project

### How to Run

**Important**: This system is built using ROS melodic and Ubuntu 18.04. Please make sure you are on the same configuration.

1. Please install AMBF in advance and make sure it is sourced correctly. Follow the instruction in this link/repo:

https://github.com/WPI-AIM/ambf

2. Clone the repository from:

https://github.com/JackHaoyingZhou/RBE501_final_project

First time cloning:
```bash
git clone --recursive https://github.com/JackHaoyingZhou/RBE501_final_project.git
```

3. Please double check your python packages. You may add corresponding packages if necessary. 

4. Visualize the model in AMBF.

For simplified model:
```bash
cd <ambf_path>/bin/<your OS>
./ambf_simulator -a <this_repository_path>/RBE501_project/models/psm1_verticle/psm_model.yaml
```

It should look like:

<p align="center">
<img src=media_and_results/psm_ambf_sample.png/>
</p>

For full model:
```bash
cd <ambf_path>/bin/<your OS>
./ambf_simulator -a <this_repository_path>/RBE501_project/models/psm_full/default.yaml
```

It should look like:

<p align="center">
<img src=media_and_results/psm_full_sample.png/>
</p>


4. Go to `<this_repository_path>/RBE501_project/scripts`

6. Run the python script for corresponding models, you can also modify the script following the comments to track different trajectories and accomplish gravity compensation

For simplified model (include gravity compensation for home position):
```bash
python3 psm_ambf.py
```

For full model:
```bash
python3 psm_full.py
```


### How to see the specific model

1. You need to download `Blender 2.8x`

2. You need to load the add on from `ambf_addon` folder into Blender.


### Scripts 

1. Python

Please read the comments in the scripts if you want to accomplish different tasks.

`psm_ambf.py` : Code for simplified model, including both inverse kinematics and dynamics control.

`psm_full.py` : Code for full model, only having inverse kinematics control.

2. Matlab

`dVRK_FKIK_ambf.m` : Code for calculating inverse kinematics of simplified model, generate a csv file recording the joint variables and path coordinates.

`dVRK_FKIK_full.m` : Code for calculating inverse kinematics of full model, generate a csv file recording the joint variables and path coordinates.

`ModelDynamics.m` : Code for calculating inverse dynamics of simplified model.


### Some development not added to the repository:

1. I would plan to include the RBDL and RBDL sever code in the repository. But it may significantly increase the difficulties of compiling. If you are interested, here are the links, please follow the instructions for installing.

https://github.com/ORB-HD/rbdl-orb

https://github.com/WPI-AIM/ambf_control_system

