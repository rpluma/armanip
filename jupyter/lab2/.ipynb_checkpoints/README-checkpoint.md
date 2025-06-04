# Lab 2: Manipulator dynamics simulation

## 1. Setup ROS 2

### 1. UMA environment

https://github.com/jmgandarias/uma_environment_tools

#### 1.1. Install UMA Environment

wsl ya instalado en la máquina con usuario_uma (misma password) y lsb_release -a no dice versión 22.04.2 LTS de Ubuntu
```
wsl --install
wsl --install Ubuntu-22.04

Enter new UNIX username: <YOUR_USERNAME>
New password:
Retype new password:
passwd: password updated successfully
Installation successful!
```

```
cd
sudo apt update
sudo apt upgrade

git clone https://github.com/jmgandarias/uma_environment_tools.git
cd uma_environment_tools/scripts
./install_uma_environment.sh

update_uma_environment

nt

create_catkin_ws

cb
```

#### 1.2. UMA Environment Organization
```
modify_uma_params
```
#### 1.3. update_uma_environment
```
update_uma_environment
```
### 2. Useful Tools

#### 2.1. Terminator
#### 2.2. create_catkin_ws

create_catkin_ws

#### 2.3. change_ros_ws

change_ros_ws
#Note that you can also use the alias 'crw'

#### 2.4. create_ros2_pkg

create_ros2_pkg

#### 2.5. nt

change_ros_ws
#Note that you can also use the alias 'crw'

#### 2.6. cb
cb

#### 2.7. cc

cc

### 3. Troubleshooting

## 2. Install UMA manipulator package

## 3. Simulate the robot dynamics

## 4. Launch the dynamics simulator node

## 5. Graphical representation

## Optional - Nice plots and vector images
