# Caltech Crater Rover

This codebase is written for the Caltech Crater 6-wheel robot.

The base of this repo was written by a JPL intern who needed a project, and found one in your subbasement. I will do my best to make this consumable to those who have experience writing software, and those who have less experience but need to try and accomplish some task.

## THINGS TO KNOW FOR CANADA COMPETITION

Always use tmux. it will start a terminal on the rover that will the program keep running even after it crashes. Login, and just type `tmux`. It will open another termianl (you'll see  a green bar at the bottom). If you get disconnect, just ssh back in and type `tmux attach` and you'll get your whole terminal back.

### How to start script on rover
```bash
$ cd ~/_CODE/crater/workspace
$ source install/setup.bash
# ros2 launch crater_bot launch_robot.launch.py
```

### How to start the script on the laptop
```bash
$ cd ~/Desktop/crater/workspace
$ source install/setup.bash
# ros2 launch crater_bot joystick.launch.py
```



### How to start the rover on 

## Things to know before getting started
- Everything here assumes you're running Ubuntu. At the time of writing your dev machine and raspberry pi are running ubuntu.

## Folders in this repository
- `ansible`: Ansible is a command line tool that lets you run "playbooks." A playbook are sets of instructions that can be run on your computer to ensure that every computer will have an identical setup. Here, I use it for scripts that you may need to run, that way you don't need to type out the commands yourself (but look through them for more details).

- `workspace`: Here is your ROS2 workspace. It's setup with the default folders. Any additional packages you want to add to this should be added as git submodules. It makes it easy to add other packages, while tracking your source code in git, without tracking all projects in git.

# Getting Started
1. Install Ansible (it's how you can run the playbooks in the `ansible` folder)
    `sudo apt install ansible`

2. Run `ansible-playbook 01.setup-ros-jazzy.yml -K`

    > [!NOTE]
    > This playbook is for Ubuntu 24.04 Jammy. If you want to run it on a newer/different operating system, review the script and make sure it's up to date.

The script is setup to run as admin. It needs `-K` in order for it to be able to answer the sudo prompt.
