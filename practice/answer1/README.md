# 問題1解答例
### 問題1.1

```bash
cd ~/catkin_ws/src/
catkin_create_pkg room_circuit_controller std_msgs rospy roscpp
```

### 問題1.2
room_circuit_controller/srcにroom_circuit_controller.cppを作成．

```cpp
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher pub;
ros::Subscriber sub;

void scanCallback(sensor_msgs::LaserScan msg)
{
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "room_circuit_controller");
    ros::NodeHandle n;

    pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    sub = n.subscribe("/scan", 1, scanCallback);

    ros::spin();
}
```