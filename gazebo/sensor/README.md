# ロボットモデルにセンサを追加

[前のページ](../spawn/)

## 概要

Gazeboではいくつかのセンサのシミュレーションも可能です．今回はロボットモデルに2D LiDARセンサを追加します．

## LiDARとは
LiDAR (Light Detection and Ranging) は周囲の障害物までの距離を計測できるセンサです．LRF (Laser Range Finder) やLRS (Laser Range Sensor) などとも呼ばれます．

LiDARと呼ぶことにすれば、2次元平面上の周囲の障害物の距離を計測できるものを2D LiDAR、3次元空間上の周囲の障害物の距離を計測できるものを3D LiDARといいます。

今回は2D LiDARセンサを追加し，ロボット周囲の2次元点群情報を取得できるようにします．

余談の欄によく使用される2D LiDARについて記載しましたが，今回はそのうちSickの2D LiDARをモデリングしていきます．（Sickは慶應に様々な支援をしてくださっているためです笑）

Sickの2D LiDARのうち小型で性能が高く移動ロボットと相性が良いものは[TiM571](https://www.sick.com/jp/ja/detection-and-ranging-solutions/2d-lidar/tim5xx/tim571-9950101s01/p/p456849)なので，今回はこちらの仕様を見ながらパラメータを合わせていきます．

## センサモデルの作成
### リンク
今まで通り，URDFでセンサのリンクを作成します．

<img src='./fig/1.png' width="500" >

|  重量  |  寸法 (長さ x 幅 x 高さ)  |
| ---- | ---- |
| 250 g, 接続ケーブルなし |  60 mm x 60 mm x 86 mm  |

[TiM571](https://www.sick.com/jp/ja/detection-and-ranging-solutions/2d-lidar/tim5xx/tim571-9950101s01/p/p456849)より引用

TiM571の寸法を見て，大まかに半径30mm, 高さ86mmの円柱に見立ることにします．また，重量は0.25kgとします．色は白や黒だと車体や車輪の色と被るため．ここではSickのロゴの色の青にしてみます．

ここまで決めれば，今まで学んだ内容で以下のURDFが記述できると思います．

```xml
  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.30" length="0.086"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <material name="blue">
        <color rgba="0.0 0.0 1.0 2.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.30" length="0.086"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </collision>
    <inertial>
        <origin xyz="0 0 0" rpy="0 0 0" />
        <mass value="0.25" />
        <inertia ixx="0.00021033333" 
            ixy="0"  ixz="0"
            iyx="0"  iyy="0.00021033333" 
            iyz="0"
            izx="0"  izy="0"  izz="0.0001125" />
    </inertial>
  </link>

  <gazebo reference="lidar_link">
    <material>Gazebo/Blue</material>
  </gazebo>
```

### ジョイント
今回は，ルンバの車体のど真ん中にLiDARを設置してみましょう．body_linkの円柱の高さが72mm, LiDARの円柱の高さが86mmなので，body_linkの重心から真上に72/2+86/2=79mmのところにlidar_linkの重心が来ます．よって以下のようにジョイントを記述します．

```xml
  <joint name="lidar_joint" type="fixed">
    <parent link="body_link"/>
    <child  link="lidar_link"/>
    <origin xyz="0 0 0.079" rpy="0 0 0" />
  </joint>
```

## GazeboのLaserプラグイン
ここで，GazeboのLaserプラグインを用いることでLiDARをシミュレーションします．以下のように書きます．なお，`<gazebo>`タグでの記述なので，先程書いた色の情報も一緒に含めました．

```xml
   <gazebo reference="lidar_link">
      <material>Gazebo/Blue</material>
      <sensor type="ray" name="lidar">
        <pose>0 0 0 0 0 0</pose>
        <visualize>false</visualize>
        <update_rate>15</update_rate>
        <ray>
          <scan>
            <horizontal>
              <samples>400</samples>
              <resolution>1</resolution>
              <min_angle>${radians(-135)}</min_angle>
              <max_angle>${radians( 135)}</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.05</min>
            <max>25.0</max>
            <resolution>0.01</resolution>
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </ray>
        <plugin name="gazebo_ros_lidar_controller" filename="libgazebo_ros_laser.so">
          <topicName>scan</topicName>
          <frameName>lidar_link</frameName>
        </plugin>
      </sensor>
    </gazebo>
```

以下にパラメータの説明を記載します．重要なものは黒字にしました．それ以外はそういうものかと思ってくだされば結構です．

- gazebo：リンク名を書いてgazeboの内容と紐づける
    - material：すでに説明した通り，色
    - sensor：センサの種類
        - **pose**：リンク座標系からみてどの位置にlaserの受光源を設置するか
        - visualize：Gazebo上でLaser光線を可視化するか否か
        - **update_rate**：センサの情報をpublishする周波数
        - ray
            - scan
                - **horizontal**：水平方向のLiDARの設定．3D LiDARの場合はverticalも記載する．
                    - samples：一周で何ポイント分取得するか．
                    - resolution：sample*resolutionが一周で何ポイント分取得するかの値らしい（[コード](https://github.com/osrf/gazebo/blob/gazebo11/gazebo/sensors/RaySensor.cc#L218-L219)）．基本はresolutionは1
                    - min_angle：前方を0 radとして，最小何radまで検出できるか
                    - max_angle：前方を0 radとして，最大何radまで検出できるか
            - **range**
                - min：検出できる最小距離 (m)
                - max：検出できる最大距離 (m)
                - resolution：何m単位で値を出力するか
            - noise
                - type：ノイズの種類
                - mean：ノイズの平均値
                - std_dev：ノイズの分散値
        - plugin
            - **topicName**：publishされるセンサデータのROSトピック名
            - **frameName**：センサデータと紐づけるframe名（リンク名）

上記のパラメータを，実際のLiDARの仕様に合わせて変更します．

<img src='./fig/3.png' width="500" >

<img src='./fig/2.png' width="500" >

角度は-135度から135度とし，ラジアン変換してパラメータを変更します．また，update_rateは15Hzとします．samplesは270/0.33≒818と計算できますが，今回は計算負荷削減のために400とします．

また，rangeのminは0.05m maxは25mとします．

## リンク

[次のページ](../spawn/)

[目次](../../)



---

## 余談
### Gazeboでシミュレーションできるセンサ例
RGBD Camera, IMU, Laser sensor (1D, 2D, 3D LiDAR), etc...
### 2D LiDAR
北陽電機，Sick, RPLiDAR, YDLiDAR, etc...

### 3D LiDAR
Velodyne, Robosense, 北陽, Sick, etc...