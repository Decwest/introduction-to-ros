# 演習問題2　解答例

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
    int looprate = 10;
    pnh.getParam("pub_string", pub_string);
    pnh.getParam("looprate", looprate);

    ros::Publisher simple_pub = nh.advertise<std_msgs::String>("my_topic", 1);
    ros::Rate loop_rate(looprate);

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

