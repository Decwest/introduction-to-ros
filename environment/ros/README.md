# ROS インストール

[前のページ](https://github.com/Decwest/introduction-to-ros)

## 概要

Ubuntu 20.04 に ROS (noetic) をインストールします．

以下に記載のコマンドを[ターミナル](https://linuxfan.info/ubuntu-open-terminal-emulator)に打っていきます．

ターミナルはCtrl+Alt+tで開けます．

また，ペーストはタッチパッドを二本指で押すことでWindowsでいう右クリックすると出てくるような画面が表示され，そこでPasteを押すことで出来ます．



## ROSのインストール

```shell
sudo apt update && \
sudo apt install curl -y && \
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - && \
sudo apt update && \
sudo apt install ros-noetic-desktop-full -y
```



## 環境設定

```shell
source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```



## ツール等インストール，設定

```shell
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool python3-osrf-pycommon python3-catkin-tools build-essential -y
```



```shell
sudo rosdep init && \
rosdep update
```



## ワークスペース作成

```shell
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin build
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```



## 動作確認

以下のコマンドでrosが立ち上がります．

```shell
roscore
```

Ctrl+Cで終了できます．



以下のコマンドで物理シミュレータであるGazeboが立ち上がります．

```shell
gazebo
```

Ctrl+Cで終了すると安全です．



VMware上で実行しており，gazeboの描画がうまくいかない場合，以下のコマンドを入力してください．

```shell
echo "export SVGA_VGPU10=0" >> ~/.bashrc
source ~/.bashrc
```



## VS Code インストール

VS Codeエディタをインストールしておくとコードの作成が楽です．

https://code.visualstudio.com/download



**推奨拡張機能**

- 日本語
- C++
- Python
- ROS



## リンク
[次のページ](https://github.com/Decwest/introduction-to-ros/blob/main/environment/ros/README.md)

[目次](https://github.com/Decwest/introduction-to-ros)



---

### こぼれ話

## ROSのバージョンについて

UbuntuとROSのバージョンは紐づいています．

基本的にはこの対応関係以外のバージョンのROSをUbuntuに入れることはできません．(Ubuntu 20.04にROS melodicを入れるなど)

| Ubuntu version | ROS version |
| -------------- | ----------- |
| 14.04          | indigo      |
| 16.04          | kinetic     |
| 18.04          | melodic     |
| 20.04          | noetic      |

そして，各ROSのバージョンに互換性はあまりありません．

例えば，今回入れたROS noeticはPython3に標準対応していますが，melodic以前は対応していません．

なので，melodic以前で開発されたrospyスクリプトをnoeticで実行するとエラーが出ることがあります．

私は別のバージョンで開発されたROSパッケージを動かす際は，動いたらラッキー程度に考えています．

