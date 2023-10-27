<p align="center">
<img src=./Images/example.gif width=45% height=45%>
</p>

# IK-MPC

![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)
![CMake](https://img.shields.io/badge/CMake-%23008FBA.svg?style=for-the-badge&logo=cmake&logoColor=white)

[![Powered by the Robotics Toolbox](https://raw.githubusercontent.com/petercorke/robotics-toolbox-python/master/.github/svg/rtb_powered.min.svg)](https://github.com/petercorke/robotics-toolbox-python)

Inverse Kinematics-based Model Predictive Control for Robot Motion Planning. Using Numerical inverse kinematics of a URDF model and Linear Quadratic Model Predictive Control (LQ-MPC), this code computes desired joint commands to follow reference path(s) that a robot needs to follow. Original MATLAB version and detailed documentation are available at [manipulator-mpc](https://github.com/twtoner/manipulator-mpc).


## Table of Contents

- [Repository Structure](#repository-structure)
- [Download Process](#download-process)
- [How to Run](#how-to-run)
- [How to Apply on Your Robot](#how-to-apply-on-your-robot)
- [ToDo Lists](#todo-lists)

---

## Repository Structure

    ├── images   
    ├── MPC 
    │   └── uncMPC.py
    ├── meshes/m0609_white
    │   ├── MF0609_0_0.dae
    │   ├── MF0609_1_0.dae
    │   ....
    ├── urdf
    │   └── m0609.urdf
    └── MPC_trajectory_following.py 

## Download Process

    git clone https://github.com/kidpaul94/IK-MPC.git
    cd IK-MPC/
    pip3 install -r requirements.txt

## How to Run

> **Note**
You need to set absolute paths for each mesh file called by `m0609.urdf`. In `MPC_trajectory_following.py`, you also need to provide a path to the urdf you just modified. Then,

     python3 MPC_trajectory_following.py
    
## How to Apply on Your Robot
> **Note**
To apply this code to your own robotic model, you need your custon urdf file. You can build the urdf using any CAD software such as SolidWorks or Fusion360. If you don't want to use a numerical IK, it is also possible to compute an analytical IK by using IKFast. More details about how to get the IKFast analytical solution, check out [ROS MoveIt](http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/ikfast/ikfast_tutorial.html).

## Citation
    @inproceedings{rtb,
    title={Not your grandmother’s toolbox--the Robotics Toolbox reinvented for Python},
    author={Corke, Peter and Haviland, Jesse},
    booktitle={2021 IEEE International Conference on Robotics and Automation (ICRA)},
    pages={11357--11363},
    year={2021},
    organization={IEEE}
    }

## ToDo Lists

| **Documentation** | ![Progress](https://progress-bar.dev/50) |
| --- | --- |

