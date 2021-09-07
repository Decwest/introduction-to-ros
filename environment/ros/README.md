# ROS インストール

[前のページ](https://github.com/Decwest/introduction-to-ros/environment/README.md)

## 概要

Ubuntu 20.04 に ROS (noetic) をインストールします．

以下に記載のコマンドを[ターミナル](https://linuxfan.info/ubuntu-open-terminal-emulator)に打っていきます．

ターミナルはCtrl+Alt+tで開けます．

また，ペーストはタッチパッドを二本指で押すことでWindowsでいう右クリックすると出てくるような画面が表示され，そこでPasteを押すことで出来ます．

[ROSの公式Wiki](http://wiki.ros.org/noetic/Installation/Ubuntu)

## ROSのインストール

```shell
sudo apt update
sudo apt install curl -y
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full -y
```

 <details><summary>各行の説明</summary><div>

- [apt](https://weblabo.oscasierra.net/ubuntu-apt/)リポジトリのパッケージリストを更新
- [curl](https://curl.se/)というツールをインストール（-yはインストール中の質問に全てYesで答えるという意味）
- ROSのaptリポジトリのURLをaptのパッケージリストに追加
- ROSのaptリポジトリにアクセスするためのキーを登録
- aptリポジトリのパッケージリストを更新
- ROSのインストール（desktop-fullはシミュレータ等含む全てをインストール）

</div></details>


## 環境設定

```shell
source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```

 <details><summary>各行の説明</summary><div>

- /opt/ros/noetic/setup.bashというbashスクリプトを実行 (source) （これによりROS関連のディレクトリにパスが通る）
- "source /opt/ros/noetic/setup.bash"という記述を~/.bashrcに追記 (echo)（~/.bashrcに記述されたコマンドが毎回ターミナル起動時に実行される）

</div></details>


## ツール等インストール，設定

```shell
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool python3-osrf-pycommon python3-catkin-tools build-essential -y
sudo rosdep init
rosdep update
```

 <details><summary>各行の説明</summary><div>

- 依存関係解決やビルドに用いるツールをインストール
- ROSパッケージの依存関係を解決するツールであるrosdepの設定（[この記事](https://qiita.com/strv/items/ccd89c55957bd8dfada0)が参考になります）

</div></details>


## ワークスペース作成

```shell
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin build
source ~/catkin_ws/devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

 <details><summary>各行の説明</summary><div>

- ~/catkin_ws/srcというディレクトリ（フォルダ）を作成
  - ~は[ホームディレクトリ](https://uxmilk.jp/27448)
  - ホームディレクトリにcatkin_wsというフォルダを作成し，さらにその中にsrcフォルダを作成
  - mkdirはmake directoryの略．-pオプションをつけると一気に複数階層分作成可能．
- ~/catkin_wsに作業ディレクトリを移動
  - cdはchange directoryの略
- ROSパッケージをビルドするコマンド
  - 初回実行時はその際にいるディレクトリをワークスペースと認識．今回はその用途．
- ~/catkin_ws/devel/setup.bashというbashスクリプトを実行（これによりROS関連のディレクトリにパスが通る）
- "source ~/catkin_ws/devel/setup.bash"という記述を~/.bashrcに追記

</div></details>


## 動作確認

以下のコマンドでROSが立ち上がります．

```shell
roscore
```

このような画面になれば成功です．

<img src='https://raw.githubusercontent.com/Decwest/introduction-to-ros/main/environment/ros/fig/1.png' width="500" >

Ctrl+Cで終了できます．


また，以下のコマンドで物理シミュレータであるGazeboが立ち上がります．

```shell
gazebo
```

バツ印よりCtrl+Cで終了すると安全です．

<img src='https://raw.githubusercontent.com/Decwest/introduction-to-ros/main/environment/ros/fig/2.png' width="500" >

なお，VMware上で実行しており，以下のようにgazeboの描画がうまくいかない（物体の輪郭が表示されない）場合，以下のコマンドを入力してください．
（物体は赤線でくくったところをクリックして引っ張ってきています）

<img src='https://raw.githubusercontent.com/Decwest/introduction-to-ros/main/environment/ros/fig/3.png' width="500" >

```shell
export SVGA_VGPU10=0
echo "export SVGA_VGPU10=0" >> ~/.profile
```

 <details><summary>各行の説明</summary><div>

- SVGA_VGPU10という名前の環境変数を宣言し，値に0を代入
  - これによりOpenGL2が使用されるようになる（デフォルトではOpenGL3．VMwareがOpenGL3に対応していないことがある）
- "export SVGA_VGPU10=0"という記述を~/.profileに追記
  - ~/.profileはログイン時に実行
  - ~/.bashrcに追記すると，仮にターミナルを介さずにGazeboが実行された場合うまくいかないため，~/.profileに追記

</div></details>

参考

[Virtual Machine not launching Gazebo](https://answers.gazebosim.org//question/13214/virtual-machine-not-launching-gazebo/)

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

基本的にはこの対応関係以外のバージョンのROSをUbuntuに入れることはできません．(Ubuntu 20.04にROS melodicを入れるなどは✖)

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

