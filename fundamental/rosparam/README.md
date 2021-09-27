# rosparam

[前のページ](../topic)

[目次](../../)



## 概要
外部からパラメータの値を受け取れるようなROSノードの書き方について説明します．

前のページで作成したpublisherにパラメータを受け取る記述を追加して，送信する文字列をパラメータで指定可能なようにしましょう．

#### my_simple_publisher.cpp
```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_simple_publisher");
    ros::NodeHandle nh;
    
    // added
    ros::NodeHandle pnh("~");
    std::string pub_string = "hello world";
    pnh.getParam("pub_string", pub_string);

    ros::Publisher simple_pub = nh.advertise<std_msgs::String>("my_topic", 1);
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        std_msgs::String msg;
        msg.data = pub_string;
        ROS_INFO("publish: %s", msg.data.c_str());
        simple_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
```

**各行の意味**
```cpp
ros::NodeHandle pnh("~");
std::string pub_string = "hello world";
pnh.getParam("pub_string", pub_string);
```
- パラメータを受け取るための新たなノードハンドルの作成 
    - "~"で初期化するとプライベートな名前空間（/ノードの名前/パラメータ名）にアクセス可能
        - 一般にパラメータは`/ノードの名前/パラメータ名`で与えられる（異なるノード間のパラメータ名衝突を避けるため）
    - 一方publisherやsubscriberを登録するノードハンドルは`ros::NodeHandle nh;`とするが，このようにするとROSデフォルトの[相対的な名前解決](http://wiki.ros.org/ja/Names)が行われる．トピック名などがこの形式をとっているためそうする．
    - [参考](https://answers.ros.org/question/309008/when-and-why-do-we-use-two-or-more-rosnodehandle-for-one-node/)
- パラメータで受け取る値を格納する変数の定義．デフォルト値を与えておくとよい．
- getParamで""で囲まれた名前のパラメータを受け取り，第二引数に渡した変数に格納する．
    - このコードの場合，もし"pub_string"というパラメータが外部から与えられなければ，pub_string変数に値の代入は行われない．よってデフォルト値を入れていないと何が起こるかわからないためデフォルト値を入れている．
    - getParamは値を受け取れた場合true，受け取れなかった場合falseを返すので，その結果を用いて例外処理を挟むことも可能．（[参考](https://docs.ros.org/en/noetic/api/roscpp/html/classros_1_1NodeHandle.html#ad25eaed5ee612a733297511aa00458e1)）


## 動作確認
### ビルド
```
catkin build
```
### 実行
```
roscore
```
したうえで別ターミナルで
```
rosrun my_ros_tutorial my_simple_publisher _pub_string:="hoge"
```
すると出力が以下のようになるはず

```shell
decwest@DESKTOP-PRFDO60:~/catkin_ws$ rosrun my_ros_tutorial my_simple_publisher _pub_string:="hoge"
[ INFO] [1632634919.572324700]: publish: hoge
[ INFO] [1632634919.672427100]: publish: hoge
[ INFO] [1632634919.772410900]: publish: hoge
[ INFO] [1632634919.872397500]: publish: hoge
[ INFO] [1632634919.972398900]: publish: hoge
```

## 演習問題2
上記のpublisherのループ周波数をパラメータで変更できるようにしよう

[解答例](./answer)

## リンク

[次のページ](../roslaunch/)

[目次](../../)



---

## 余談
### Pythonでの実装


### パラメータサーバについて
ROSはパラメータをパラメータサーバというもので管理しています．パラメータの値と名前が保存されていて，適宜パラメータの名前と値をサーバーに登録したり，読み出したりします．
興味のある方は[このROS Wiki](http://wiki.ros.org/Parameter%20Server)を見てください．

パラメータの登録には[コマンドラインツール](http://wiki.ros.org/rosparam)，rosrunのオプション（今回扱ったやつ），launchファイルでの指定（次のページで扱う）等があります．

## Dynamic reconfigure
ROSのパラメータは基本的には静的で，一度登録したら値の書き換えができません．しかし，dynamic reconfigureというものを用いれば途中で値の変更が可能な動的なパラメータを作成することができます．
[こちら](https://qiita.com/srs/items/3adcc5898955a6aa1631)がわかりやすいです．

### 名前空間について
興味のある方は[このROS Wiki](http://wiki.ros.org/Names)を見てください．

### rosrunオプション
今回，以下のようにしてノードにパラメータを渡したと思います．
```
rosrun my_ros_tutorial my_simple_publisher _pub_string:="hoge"
```
ここで，_の数によって何の名前に関するものを設定するのかが変わります．
- __（2個）

    ノード名に関する変更になります．`__name:=(変更後のnodeの名前)`とします．

- _（1個）

    パラメータに関する変更になります．`_(値を変更したいパラメータ名):=(パラメータの値)`とします．
    なお，パラメータ名は勝手にプライベートな名前空間になるので，ノード側は"~"で初期化したノードハンドルで受け取れます．

- （0個）

    トピック名に関する変更になります．`(元のトピック名):=(変更後のトピック名)`とします．