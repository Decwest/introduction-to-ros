# Gazebo上にロボットをスポーンさせる

[前のページ](../actuator/)

## 概要

Gazebo上にロボットをスポーンさせるlaunchファイルを書きます．

## launchファイル作成

my_urdf_tutorial配下にlaunchディレクトリを作成し，robot_simulation.launchという名前のlaunchファイルを作成します．

```xml
<?xml version="1.0"?>
<launch>
  <param name="robot_description" textfile="$(find my_urdf_tutorial)/urdf/roomba_sim.urdf"/>

  <include file="$(find gazebo_ros)/launch/empty_world.launch"/>

  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-param robot_description -x 0.0 -y 0.0 -z 0.5 -R 0 -P 0 -Y 0 -urdf -model roomba_sim" />

  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" output="screen" />
  
  <node name="rviz" pkg="rviz" type="rviz"/>

</launch>
```

各行解説
```xml
  <param name="robot_description" textfile="$(find my_urdf_tutorial)/urdf/roomba_sim.urdf"/>
```

```xml
  <include file="$(find gazebo_ros)/launch/empty_world.launch"/>
```

```xml
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-param robot_description -x 0.0 -y 0.0 -z 0.5 -R 0 -P 0 -Y 0 -urdf -model roomba_sim" />
```

```xml
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" output="screen" />
```

```xml
  <node name="rviz" pkg="rviz" type="rviz"/>
```

## launch
```bash
roslaunch my_urdf_tutorial robot_simulation.launch
```

## ロボットを操作してみる
```bash
sudo apt install ros-noetic-teleop-twist-keyboard
```

## リンク

[次のページ](../spawn/)

[目次](../../)



---

## 余談