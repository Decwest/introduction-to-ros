# URDFにアクチュエータを追加

[前のページ](../physical_quantity/)

## 概要

ROSトピックで車輪を回せるようにするために，roomba_sim.urdfにアクチュエータの記述を追加します．ルンバは2つの独立した車輪で移動する差動二輪ロボットなので，今回はGazeboのdifferential_drive_controllerというプラグインを用いてROSとGazeboを接続します．

参考：[Gazebo Tutorial](http://gazebosim.org/tutorials?tut=ros_gzplugins)

---
プチ余談：

Gazeboの回転ジョイントをROSトピックで直接回転させることも可能です．ただ，発展的な内容なのと，差動二輪ロボットの運動学についても述べる必要があるため，今回はdifferential_drive_controller ModelPluginを用いてショートカットします．時間があればジョイントを回す方法及びジョイントのステータスを取得する方法について余談で記載できたらと思います．

---

## differential_drive_controller

roomba_sim.urdfの末尾に以下を追記します．Gazebo Tutorialからコピペしたのがベースです．ただし，Gazebo Tutorialに載っている情報が少し古かったので，私がコードを掘って一部修正しました．ROSは結構こういうことがあります．オープンソースなので，Wiki通りに動かない場合はコードを掘ることを推奨します．興味のある人は説明も読んでみてください．

参考：コード

[gazebo_ros_diff_drive.cpp](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_plugins/src/gazebo_ros_diff_drive.cpp)

[gazebo_ros_diff_drive.h](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_plugins/include/gazebo_plugins/gazebo_ros_diff_drive.h)

```xml
<gazebo>
  <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">

    <!-- Plugin update rate in Hz, defaults to 100 Hz -->
    <updateRate>30</updateRate>

    <!-- Name of left joint, defaults to `left_joint` -->
    <leftJoint>wheel_joint_left</leftJoint>

    <!-- Name of right joint, defaults to `right_joint` -->
    <rightJoint>wheel_joint_right</rightJoint>

    <!-- The distance from the center of one wheel to the other, in meters, defaults to 0.34 m -->
    <wheelSeparation>0.332</wheelSeparation>

    <!-- Diameter of the wheels, in meters, defaults to 0.15 m -->
    <wheelDiameter>0.072</wheelDiameter>

    <!-- Wheel acceleration, in rad/s^2, defaults to 0.0 rad/s^2 -->
    <wheelAcceleration>1.0</wheelAcceleration>

    <!-- Maximum torque which the wheels can produce, in Nm, defaults to 5 Nm -->
    <wheelTorque>10</wheelTorque>

    <!-- Topic to receive geometry_msgs/Twist message commands, defaults to `cmd_vel` -->
    <commandTopic>cmd_vel</commandTopic>

    <!-- Topic to publish nav_msgs/Odometry messages, defaults to `odom` -->
    <odometryTopic>odom</odometryTopic>

    <!-- Odometry frame, defaults to `odom` -->
    <odometryFrame>odom</odometryFrame>

    <!-- Robot frame to calculate odometry from, defaults to `base_footprint` -->
    <robotBaseFrame>base_link</robotBaseFrame>

    <!-- Odometry source, encoder or world, defaults to world -->
    <odometrySource>world</odometrySource>

    <!-- Set to 1 to create odom publisher, defaults to 1 -->
    <publishTf>1</publishTf>

    <!-- Set to true to publish transforms for the wheel links, defaults to false -->
    <publishWheelTF>true</publishWheelTF>

    <!-- Set to true to publish transforms for the odometry, defaults to true -->
    <publishOdomTF>true</publishOdomTF>

    <!-- Set to true to publish sensor_msgs/JointState on /joint_states for the wheel joints, defaults to false -->
    <publishWheelJointState>true</publishWheelJointState>

    <!-- Set to true to swap right and left wheels, defaults to true -->
    <legacyMode>false</legacyMode>

    <!-- DebugMode or not, na or Debug, defaults to na -->
    <rosDebugLevel>na</rosDebugLevel>
  </plugin>
</gazebo>
```

補足
- wheel_separationは車輪の右端から左端までの距離を指定
- commandTopicはロボットに送る速度指令のROSトピックの名前で，デフォルトのままcmd_vel．型は`geometry_msgs/Twist`．`geometry_msgs/Twist`型のROSトピックcmd_velは移動ロボットの速度指令において必ず出てくるのでぜひ覚えましょう．

次のページで，今まで作成したロボットをGazebo上にスポーンさせてみましょう．

## リンク

[次のページ](../spawn/)

[目次](../../)



---

## 余談
### ros_controlについて
Gazeboのros_controlプラグインを用いてGazeboとROSをつなぎ，ROS側でros_controlを用いた制御をかけることでつなぎこむことも可能です．

参考になるリンク：

[Gazebo + ROS で自分だけのロボットをつくる 5. GazeboとROSの連携](https://qiita.com/RyodoTanaka/items/6fa7e45f98b55376a95b)

[Controller と HardwareInterface との間の処理の仕組み（1. ロボットモデルの定義と登録）](https://qiita.com/MoriKen/items/613635b90f3a98042dc5)


[ros_controls/ros_controllers の制御の仕組み (position/effort/velocity_controllers の基礎)](https://qiita.com/MoriKen/items/78b0ad8c1eae257646dd)



### Gazeboの回転ジョイントをROSトピックで制御する
トピック名：`/名前空間/ros_controlのコントローラ名/command`

参考になるリンク：
[ROS講座27 gazeboでjointを動かす](https://qiita.com/srs/items/8868a8bef3752c3464a2)

### Gazeboの回転ジョイントのステータスをROSトピックで取得する
joint_state_publisherで一括取得
### odometry
車輪型移動ロボットにおける車輪の回転角からそれぞれの車輪の移動量を求め、ロボットの位置を推定する手法の総称をオドメトリ (odometry) と言います．
### Frame
TF