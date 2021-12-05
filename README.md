# ROS (Robot Operating System) 入門

## 概要
今後ROSについて自力で学習できるような環境と地力を獲得することを目標としています．

内容は環境構築，ROSの基本的な事項，シミュレーションの実行から構成されています．

**大まかな内容**

+ ROSが動作する環境の構築
  - Ubuntu 20.04 環境の用意
  - ROS (noetic) のインストール
+ ROSの基本的な事項
  - トピック通信
  - rosparam
  - roslaunch
  - コマンドラインツール
+ Rvizで可視化
  - ロボットモデルをURDF形式で記述
  - Rvizでロボットモデルの可視化
+ Gazeboを用いたシミュレーション
  - LiDARを1台積んだ差動二輪ロボットモデルをURDF形式で記述
  - Gazeboでシミュレーション
  - Rvizでロボットモデルとトピックを可視化


## 環境構築

### [Ubuntu環境の用意](./environment/)

Windows, Mac PCを所持している方はVMwareで仮想マシンを構築することで環境を用意します．

### [ROSのインストール](./environment/ros/)

Ubuntu環境にROSをインストールします．

## ROSの基本的な事項

### [ROSとは](./ros/about/)
ROSとはなにか簡単に説明します。

### [トピック通信実践](./ros/topic/)
ROSの基本的な概念であるトピック通信について実装を通して理解します。

### [rosparam](./ros/rosparam/)
外部からパラメータの値を受け取れるようなROSノードの書き方について説明します．

### [roslaunch](./ros/roslaunch/)
複数のノードを同時に起動したりそれぞれにパラメータを渡したりすることを一つのファイルで記述可能なroslaunchについて説明します．

### [コマンドラインツール](./ros/command/)
ターミナル上で使用可能な，ROS環境の解析に有用なコマンドラインツールをいくつか紹介します．


## Rvizで可視化

### [Rvizとは](./rviz/about)
ROSに付随する可視化ツールであるRvizについて，できることを簡単に説明します．

### [ロボットモデルの作成：URDF](./rviz/urdf)
Rvizで表示できるロボットモデルはURDFという形式で記述されます．URDFの考え方と文法について説明します．

### [URDF形式でロボットモデル作成演習](./rviz/practice)
前章で学んだことを活かして，ルンバのような差動二輪ロボットのモデルをURDFで記述する演習を行います．

## Gazeboを用いたシミュレーション
### [Gazeboとは](./gazebo/about)
ROSと相性の良い物理シミュレータであるGazeboについて，できることを簡単に説明します．

### [URDFに物理量を追加](./gazebo/physical_quantity)
前章で作成したロボットモデルにGazeboシミュレーション時に必要な物理量を追加します．

### [URDFにアクチュエータを追加](./gazebo/actuator)
ROSトピックでロボットの車輪を回せるようにするために，アクチュエータの記述を追加し，ROSとGazeboをつなぎこみます．

### [Gazebo上にロボットをスポーン](./gazebo/spawn)
Gazebo上にロボットをスポーンさせるlaunchファイルを書きます．

### [ロボットモデルにセンサを追加](./gazebo/sensor)
Gazeboではいくつかのセンサのシミュレーションも可能です．今回はロボットモデルに2D LiDARセンサを追加します．

## [演習：部屋を巡回するルンバのROSプログラミング](./practice/)
今までの章でROSの基礎，2D LiDARを積んだ移動ロボットのシミュレーションモデルの作成を行いました．本章では，2D LiDARで取得した周囲の障害物情報を用いて，障害物にぶつからないように部屋を巡回する知能ロボットのコード作成をROSを用いて行います．

---
参考

本サイトで作成するコードの完成形は以下のリポジトリにまとめてあります．

[https://github.com/Decwest/introduction-to-ros-pkgs](https://github.com/Decwest/introduction-to-ros-pkgs)