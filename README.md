<div id="top"></div>

<!-- 

# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                         #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso - Mikel.Bueno-Viso@cranfield.ac.uk                         #
#           Dr. Seemal Asif      - s.asif@cranfield.ac.uk                               #
#           Prof. Phil Webb        - p.f.webb@cranfield.ac.uk                           #
#                                                                                       #
#  Date: April, 2023.                                                                   #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA (2023) ROS 2: Sim-to-Real Robot Control. URL: https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl.

-->

<!--

  README.md TEMPLATE obtined from:
      https://github.com/othneildrew/Best-README-Template
      AUTHOR: OTHNEIL DREW 

-->

<!-- HEADER -->
<br />
<div align="center">
  <a>
    <img src="media/header.jpg" alt="header" width="1228" height="287">
  </a>

  <br />

  <h2 align="center">ROS 2: Sim-to-Real Robot Control</h2>

  <p align="center">
    IFRA (Intelligent Flexible Robotics and Assembly) Group
    <br />
    Centre for Robotics and Assembly
    <br />
    Cranfield University
  </p>
</div>

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about">About</a>
      <ul>
        <li><a href="#intelligent-flexible-robotics-and-assembly-group">IFRA-Cranfield Research Group</a></li>
        <li><a href="#ros2_simrealrobotcontrol-repository">ros2_SimRealRobotControl Repository</a></li>
      </ul>
    </li>
    <li>
      <a href="#documentation">Documentation</a>
    </li>
    <li>
      <a href="#supported-robots-and-end-effectors">Supported Robots and End-Effectors</a>
    </li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#cite-our-work">Cite our work</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>

<br />

<!-- ABOUT THE PROJECT -->
## About

### Intelligent Flexible Robotics and Assembly Group

The IFRA (Intelligent Flexible Robotics and Assembly) Group is part of the Centre for Robotics and Assembly at Cranfield University.

IFRA Group pushes technical boundaries. At IFRA we provide high tech automation & assembly solutions, and we support smart manufacturing with Smart Industry technologies and solutions. Flexible Manufacturing Systems (FMS) are a clear example. They can improve overall operations and throughput quality by adapting to real-time changes and situations, supporting and pushing the transition towards flexible, intelligent and responsive automation, which we continuously seek and support.

The IFRA Group undertakes innovative research to design, create and improve Intelligent, Responsive and Flexible automation & assembly solutions, and this series of GitHub repositories provide background information and resources of how these developments are supported.

__SOCIAL MEDIA__:

IFRA-Cranfield:
- YouTube: https://www.youtube.com/@IFRACranfield
- LinkedIn: https://www.linkedin.com/in/ifra-cranfield/

Centre for Robotics and Assembly:
- Instagram: https://www.instagram.com/cranfieldrobotics/
- Facebook: https://www.facebook.com/cranfieldunirobotics/
- YouTube: https://www.youtube.com/@CranfieldRobotics
- LinkedIn: https://www.linkedin.com/company/cranfieldrobotics/
- Website: https://www.cranfield.ac.uk/centres/centre-for-robotics-and-assembly 


### ros2_SimRealRobotControl Repository

The IFRA-Cranfield/ros2_SimRealRobotControl GitHub repository is a comprehensive framework designed to facilitate seamless integration of robots into both simulated and real-world environments using ROS 2. It provides a modular setup that allows for the easy deployment of various robot configurations, along with their corresponding controllers and end-effectors, without needing to redefine core parameters. The repository is built to support multiple robots, such as the ABB IRB-120, by enabling the use of ROS 2 packages that handle simulation, control, and MoveIt!2 for robot motion planning.

This repository is ideal for robotics researchers and developers who want to streamline the process of setting up robot environments for simulation and real-world tasks. It supports both Gazebo-based simulation for testing robot setups and MoveIt!2 for controlling robots in either virtual or physical environments. By organizing key robot and end-effector parameters in a modular way, it offers a flexible approach that makes it easy to switch between different robots, configurations, or tasks, accelerating both development and testing processes in industrial automation, robotics research, and advanced manufacturing systems.

__VIDEO: Simulation and Control of an ABB-IRB120 using ROS2__

[![Alt text](https://img.youtube.com/vi/qaowbdYvG2M/0.jpg)](https://www.youtube.com/watch?v=qaowbdYvG2M)

__VIDEO: Simulation and Control of a Universal Robots - UR3 using ROS2__

[![Alt text](https://img.youtube.com/vi/grhYzt0wf8c/0.jpg)](https://www.youtube.com/watch?v=grhYzt0wf8c)

<p align="right">(<a href="#top">back to top</a>)</p>

<!-- DOCUMENTATION -->
## Documentation

For detailed information on installation, usage, and requirements, please refer to the following documentation files available in this repository:

- [Installation.md](https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl/tree/humble/instructions/Installation.md): Instructions for setting up and installing the required dependencies.
- [README.md](https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl/tree/humble/packages) inside the /packages folder: Specific details about individual ROS 2 packages and their configurations.
- [ROS2EnvironmentLaunch.md](https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl/tree/humble/instructions/ROS2EnvironmentLaunch.md): Instructions for launching ROS 2 simulation and control environments.
- [RobotOperation.md](https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl/tree/humble/instructions/RobotOperation.md): Steps to operate robots in both simulated and real environments.
- [Program Execution.md](https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl/tree/humble/instructions/ProgramExecution.md): Guidelines for executing the programs in this repository.

<p align="right">(<a href="#top">back to top</a>)</p>

<!-- USAGE EXAMPLES -->
## Supported Robots and End-Effectors

The Simulation & Control packages of the following Robots are currently included in our repository:

- ABB IRB-120.
- ABB IRB-1200.
- ABB IRB-6640.
- Universal Robots UR3.
- Universal Robots UR5.
- Universal Robots UR10e.
- Universal Robots UR16e.
- KUKA LBR-iiwa.

The following end-effectors are supported in ros2srrc:

- Schunk EGP-64 Parallel Gripper.
- Zimmer GPP5010NC Parallel Gripper.
- Robotiq HandE Parallel Gripper.
- Robotiq 2f-85 Parallel Gripper.
- Custom Vacuum Gripper (Cranfield University).
- Custom Vacuum Gripper (AMRC-Sheffield).

<p align="right">(<a href="#top">back to top</a>)</p>

<!-- CONTRIBUTING -->
## Contributing

This repo contains only a few number of robots, end-effectors and simulation environments (layouts). Please do let us know if you wish to include a specific robot/end-effector or application into ros2_SimRealRobotControl!

The exact same thing for the ROS2 Robot Actions/Triggers: A few number of robot movements have been implemented, therefore please do let us know if you have any ideas of a potential Robot motions for our repo!

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**. If you have a suggestion that would make this better, or you find a solution to any of the issues/improvements presented above, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks you very much!

<p align="right">(<a href="#top">back to top</a>)</p>

<!-- LICENSE -->
## License

<p>
  Intelligent Flexible Robotics and Assembly Group
  <br />
  Created on behalf of the IFRA Group at Cranfield University, United Kingdom
  <br />
  E-mail: IFRA@cranfield.ac.uk 
  <br />
  <br />
  Licensed under the Apache-2.0 License.
  <br />
  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0
  <br />
  <br />
  <a href="https://www.cranfield.ac.uk/">Cranfield University</a>
  <br />
  School of Aerospace, Transport and Manufacturing (SATM)
  <br />
    <a href="https://www.cranfield.ac.uk/centres/centre-for-robotics-and-assembly">Centre for Robotics and Assembly</a>
  <br />
  College Road, Cranfield
  <br />
  MK43 0AL, Bedfordshire, UK
  <br />
</p>

<p align="right">(<a href="#top">back to top</a>)</p>

<!-- CITE OUR WORK -->
## Cite our work

<p>
  You can cite our work with the following statement:
  <br />
  IFRA-Cranfield (2023) ROS 2 Sim-to-Real Robot Control. URL: https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl.
</p>

<p align="right">(<a href="#top">back to top</a>)</p>

<!-- CONTACT -->
## Contact

<p>
  Mikel Bueno Viso - Research Assistant in Intelligent Automation at Cranfield University
  <br />
  E-mail: Mikel.Bueno-Viso@cranfield.ac.uk
  <br />
  LinkedIn: https://www.linkedin.com/in/mikel-bueno-viso/
  <br />
  Profile: https://www.cranfield.ac.uk/people/mikel-bueno-viso-32884399
  <br />
  <br />
  Dr. Seemal Asif - Lecturer in Artificial Intelligence and Robotics at Cranfield University
  <br />
  E-mail: s.asif@cranfield.ac.uk
  <br />
  LinkedIn: https://www.linkedin.com/in/dr-seemal-asif-ceng-fhea-miet-9370515a/
  <br />
  Profile: https://www.cranfield.ac.uk/people/dr-seemal-asif-695915
  <br />
  <br />
  Professor Phil Webb - Professor of Aero-Structure Design and Assembly at Cranfield University
  <br />
  E-mail: p.f.webb@cranfield.ac.uk
  <br />
  LinkedIn: https://www.linkedin.com/in/phil-webb-64283223/
  <br />
  Profile: https://www.cranfield.ac.uk/people/professor-phil-webb-746415 
  <br />
</p>

<p align="right">(<a href="#top">back to top</a>)</p>

<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

* [README.md template - Othneil Drew](https://github.com/othneildrew/Best-README-Template).
* [ROS 2 Documentation - Humble](https://docs.ros.org/en/humble/index.html).
* [PicNik Robotics - MoveIt!2 Documentation](https://moveit.picknik.ai/humble/index.html).
* [ABB - ROS Repositories](http://wiki.ros.org/abb).
* [ABB - ROS 2 Driver (PickNik Robotics)](https://github.com/PickNikRobotics/abb_ros2).
* [Universal Robots - ROS 2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver).

<p align="right">(<a href="#top">back to top</a>)</p>
