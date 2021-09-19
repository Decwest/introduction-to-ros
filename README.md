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
  - roslaunch
  - コマンドラインツール
+ Rvizで可視化
  - ロボットモデルをUEDF形式で記述
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

### [ROSとは](./fundamental/about/)
ROSとはなにか簡単に説明します。

### [トピック通信実践](./fundamental/topic/)
ROSの基本的な概念であるトピック通信について実装を通して理解します。

### [roslaunch](./fundamental/roslaunch/)

### [コマンドラインツール](./fundamental/command/)

## Rvizで可視化

## Gazeboを用いたシミュレーション
