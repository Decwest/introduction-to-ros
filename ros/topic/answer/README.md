# 演習問題1　解答例

my_simple_publisher.cppの

```cpp
msg.data = "hello world!";
```
を
```cpp
msg.data = "introduction-to-ros!";
```

に変更し，msgのデータを書き換えます．
その後
```
catkin build
```
を実行し，実行ファイルを生成し直しましょう．

#### my_simple_publisher.cpp
```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_simple_publisher");
    ros::NodeHandle nh;
    ros::Publisher simple_pub = nh.advertise<std_msgs::String>("my_topic", 1);
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        std_msgs::String msg;
        msg.data = "introduction-to-ros!";
        ROS_INFO("publish: %s", msg.data.c_str());
        simple_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
```