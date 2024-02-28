# A ROS Package for Legolas

Legolas is a mini bipedal robot inspired by the Cassie by Agility Robotics. The project aims to provide an open-source platform for learning and applying reinforcement learning in robotics.

<p align="center">
  <a href="assets/Walking_Test_3.mp4"><img src="assets/Walking_Test_3.gif" alt="Video 3" width="800"></a>
</p>

<p align="center">
  <a href="assets/Walking_Test_1.mp4"><img src="assets/Walking_Test_1.gif" alt="Video 1" width="800"></a>
</p>

## Repository Contents

This repository contains a ROS package, legolas_biped, to run Legolas. 

The CADs for the robot is available at [Legolas - an open source Biped](https://github.com/daviddoo02/Legolas-an-open-source-biped).

### Built With

* Ubuntu 20.04.6 LTS
* ROS Noetic
* Python

## Getting Started

1. **Clone this repository into your workspace:**

    ```bash
    git clone https://github.com/daviddoo02/Legolas_Controller.git
    ```

2. **In the root of the repository, run catkin_make to build the packages:**

    ```bash
    catkin_make
    ```

3. **Launch the walking gait:**

    ```bash
    roslaunch legolas_biped Gait_Demo.launch
    ```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Inspired by the Cassie robot by Agility Robotics.
