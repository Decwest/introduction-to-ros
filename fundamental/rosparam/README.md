# rosparam

[前のページ](../../)

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
パラメータを受け取るための新たなノードハンドルの作成

getParamでその名前のパラメータを受け取り

変数にはデフォルトで値を入れておいたほうが良い．値が入っていないと未定義動作を引き起こす

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
rosrun my_ros_tutorial my_simple_publisher pub_string:="hoge"
```
すると出力が以下のようになるはず

```
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

### 名前空間について