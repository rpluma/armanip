# Setup Entorno ROS2

## Enlaces

- (UMA Environment Tools)[https://github.com/jmgandarias/uma_environment_tools]
- (UMA Manipulator Package)[github.com/jmgandarias/uma_arm_description.git]
- (Arm Control Package)[https://github.com/jmgandarias/uma_arm_control.git]

## Dependencias
```bash
sudo apt install ros-${ROS_DISTRO}-xacro
sudo apt install ros-${ROS_DISTRO}-gazebo-ros-pkgs
sudo  apt install ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers ros-${ROS_DISTRO}-gazebo-ros2-control
sudo apt-get install -y ros-${ROS_DISTRO}-joint-state-publisher-gui ros-${ROS_DISTRO}-rviz2
```

## Git
```bash
git rm --cached -r install/
git config --global user.mail "06..."
git config --global user.name "Ric"
git config --global credential.helper store

```
## Aplicaciones instaladas

- RViz2: 3D visualization tool
- Gazebo: Simular dinámica del brazo

# Uso del entorno

## Comandos útiles

```bash
# Crear workspace de ROS (pide el nombre)
create_catkin_ws

# Cambiar a la carpeta del ws (después hay que hacer cd src)
dcw

# Compilar el workspace (desde src)
cb

# Lanzar RViz2
ros2 launch uma_arm_description uma_arm_visualization.launch.py

# Mover articulaciones
ros2 run joint_state_publisher_gui joint_state_publisher_gui

# Gestor de archivos
nautilus



update_uma_environment
```

## 
## Atajos de teclado Terminator

```
- Ctrs Shift o: dividir el terminal hOrizontalmente
- Ctrs Shift e: dividir el terminal vErticalmente
- Ctrl c: crerar todos los terminales

```