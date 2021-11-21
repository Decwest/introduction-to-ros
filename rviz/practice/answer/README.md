# ルンバのモデル作成：解答例

### リンク作成
1. base_linkを作成する

    ```xml
    <link name="base_link"/>
    ```

1. 円柱でルンバの車体body_linkを作成する
    - 半径150mm, 高さ72mmの円柱body_link：白色

    ```xml
    <link name="body_link">
        <visual>
        <geometry>
            <cylinder radius="0.15" length="0.072"/>
        </geometry>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <material name="white">
            <color rgba="1.0 1.0 1.0 1.0"/>
        </material>
        </visual>
    </link>
    ```

1. 球でルンバのキャスターball_linkを作成する
    - 半径10mmのキャスターball_link_front, ball_link_back：白色

    ```xml
    <link name="ball_link_front">
        <visual>
        <geometry>
            <sphere radius="0.01"/>
        </geometry>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <material name="white">
            <color rgba="1.0 1.0 1.0 1.0"/>
        </material>
        </visual>
    </link>

    <link name="ball_link_back">
        <visual>
        <geometry>
            <sphere radius="0.01"/>
        </geometry>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <material name="white">
            <color rgba="1.0 1.0 1.0 1.0"/>
        </material>
        </visual>
    </link>
    ```

1. 円柱でルンバの車輪wheel_linkを作成する
    - 半径36mm, 高さ16mmの円柱wheel_link_right, wheel_link_left：黒色

    ```xml
    <link name="wheel_link_right">
        <visual>
        <geometry>
            <cylinder radius="0.036" length="0.016"/>
        </geometry>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <material name="black">
            <color rgba="0.0 0.0 0.0 1.0"/>
        </material>
        </visual>
    </link>

    <link name="wheel_link_left">
        <visual>
        <geometry>
            <cylinder radius="0.036" length="0.016"/>
        </geometry>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <material name="black">
            <color rgba="0.0 0.0 0.0 1.0"/>
        </material>
        </visual>
    </link>
    ```

### ジョイント作成
1. base_linkとbody_linkの固定ジョイントbody_jointを作成する

    ```xml
    <joint name="body_joint" type="fixed">
        <parent link="base_link"/>
        <child  link="body_link"/>
        <origin xyz="0 0 0.046" rpy="0 0 0" />
    </joint>
    ```

1. body_linkとball_linkの固定ジョイントball_joint_front, ball_joint_backを作成する

    ```xml
    <joint name="ball_joint_front" type="fixed">
        <parent link="body_link"/>
        <child  link="ball_link_front"/>
        <origin xyz="0.11 0 -0.036" rpy="0 0 0" />
    </joint>

    <joint name="ball_joint_back" type="fixed">
        <parent link="body_link"/>
        <child  link="ball_link_back"/>
        <origin xyz="-0.11 0 -0.036" rpy="0 0 0" />
    </joint>
    ```

1. body_linkとwheel_linkの回転ジョイントwheel_joint_right, wheel_joint_leftを作成する
（回転の正方向がロボットの前進方向になるようにaxisのxyzを調整しています．ロボットの前方に対する右左の向き注意！）

    ```xml
    <joint name="wheel_joint_right" type="continuous">
        <parent link="body_link"/>
        <child  link="wheel_link_right"/>
        <origin xyz="0 -0.158 -0.01" rpy="1.570796326794897 0 0" />
        <axis xyz="0 0 -1" />
    </joint>

    <joint name="wheel_joint_left" type="continuous">
        <parent link="body_link"/>
        <child  link="wheel_link_left"/>
        <origin xyz="0 0.158 -0.01" rpy="1.570796326794897 0 0" />
        <axis xyz="0 0 -1" />
    </joint>
    ```

全体の解答

```xml
<robot name="roomba">

  <link name="base_link"/>

  <link name="body_link">
    <visual>
      <geometry>
        <cylinder radius="0.15" length="0.072"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <material name="white">
        <color rgba="1.0 1.0 1.0 1.0"/>
      </material>
    </visual>
  </link>

  <link name="ball_link_front">
    <visual>
      <geometry>
        <sphere radius="0.01"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <material name="white">
        <color rgba="1.0 1.0 1.0 1.0"/>
      </material>
    </visual>
  </link>

  <link name="ball_link_back">
    <visual>
      <geometry>
        <sphere radius="0.01"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <material name="white">
        <color rgba="1.0 1.0 1.0 1.0"/>
      </material>
    </visual>
  </link>

  <link name="wheel_link_right">
    <visual>
      <geometry>
        <cylinder radius="0.036" length="0.016"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <material name="black">
        <color rgba="0.0 0.0 0.0 1.0"/>
      </material>
    </visual>
  </link>

  <link name="wheel_link_left">
    <visual>
      <geometry>
        <cylinder radius="0.036" length="0.016"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <material name="black">
        <color rgba="0.0 0.0 0.0 1.0"/>
      </material>
    </visual>
  </link>
  
  <joint name="body_joint" type="fixed">
    <parent link="base_link"/>
    <child  link="body_link"/>
    <origin xyz="0 0 0.046" rpy="0 0 0" />
  </joint>

  <joint name="ball_joint_front" type="fixed">
    <parent link="body_link"/>
    <child  link="ball_link_front"/>
    <origin xyz="0.11 0 -0.036" rpy="0 0 0" />
  </joint>

  <joint name="ball_joint_back" type="fixed">
    <parent link="body_link"/>
    <child  link="ball_link_back"/>
    <origin xyz="-0.11 0 -0.036" rpy="0 0 0" />
  </joint>

  <joint name="wheel_joint_right" type="continuous">
    <parent link="body_link"/>
    <child  link="wheel_link_right"/>
    <origin xyz="0 -0.158 -0.01" rpy="1.570796326794897 0 0" />
    <axis xyz="0 0 -1" />
  </joint>

  <joint name="wheel_joint_left" type="continuous">
    <parent link="body_link"/>
    <child  link="wheel_link_left"/>
    <origin xyz="0 0.158 -0.01" rpy="1.570796326794897 0 0" />
    <axis xyz="0 0 -1" />
  </joint>

</robot>
```
