# enpm702-summer-2025-ROS
Repository for ROS packages

## Lecture 9
- ROS Overview
- ROS setup
- Publishers
- Subscribers

## Lecture 10
- Launch files
- Parameters
- Name remapping
- Simulation (Gazebo) and Visualization (RViz)

### Install Gazebo Harmonic

- See [Gazebo Harmonic Installation](https://gazebosim.org/docs/harmonic/install_ubuntu/)
- After installation, check Gazebo Harmonic is installed: `gz sim` (Ctrl + C to terminate Gazebo)
    
### Build Turtlebot3 Packages for Jazzy
- Create a new ROS workspace: 
```bash
mkdir -p ~/turtlebot3_ws/src
```
- Clone the Turtlebot repository:
```bash
cd ~/turtlebot3_ws/src/
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```
- Install missing dependencies:
```bash
cd ~/turtlebot3_ws
rosdep install --from-paths src --ignore-src -r -y
```
- Build packages:
```bash
cd ~/turtlebot3_ws && colcon build --symlink-install
```

### Sourcing Workspaces

You have two workspaces:
 - `ros702_ws` contains class packages.
 - `turtlebot3_ws` contains packages related to the Turtlebot.
 - Edit your bash shell script (`.bashrc` or `.zshrc`) function to source multiple `setup.bash` (`setup.zsh`) files in the following order:
    1. Source `setup.bash` (`setup.zsh`) from ROS
    2. Source`setup.bash` (`setup.zsh`) from `turtlebot3_ws`
    3. Source `setup.bash` (`setup.zsh`) from `ros702_ws`
 - `.bashrc`: Edit the following function with your custom paths:
 ```bash
 function ros702 {
    source /opt/ros/jazzy/setup.bash
    export TURTLEBOT3_MODEL=waffle
    ros702_ws="<path-to>/ros702_ws"
    turtlebot3_ws="<path-to>/turtlebot3_ws"
    source "${turtlebot3_ws}/install/setup.bash"
    source "${ros702_ws}/install/setup.bash"
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
    cd ${ros702_ws}
}
 ```
  - `.zshrc`: Edit the following function with your custom paths:
 ```bash
function ros702_ws {
  source /opt/ros/jazzy/setup.zsh
  export TURTLEBOT3_MODEL=waffle
  ros702_ws='/home/zeidk/ros702_ws'
  turtlebot3_ws='/home/zeidk/turtlebot3_ws'
  source ${turtlebot3_ws}/install/setup.zsh
  source ${ros702_ws}/install/setup.zsh
  eval "$(register-python-argcomplete ros2)"
  eval "$(register-python-argcomplete colcon)"
  cd ${ros702_ws}
}
 ```

### Testing Simulation with Robot

1. Restart your terminal
2. Run `ros2 launch turtlebot3_gazebo empty_world.launch.py`


