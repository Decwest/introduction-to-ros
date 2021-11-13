# 問題3解答例
## 問題3.1

以下をmain関数に追加します．

```cpp
    ros::NodeHandle pnh("~");

    pnh.getParam("velocity", velocity);
    pnh.getParam("angular_velocity", angular_velocity);
    pnh.getParam("avoid_distance", avoid_distance);
    pnh.getParam("avoid_angle", avoid_angle);
```

全体像

```cpp
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher pub;
ros::Subscriber sub;

// variables
float velocity = 0.2;          // 直進速度
float angular_velocity = 1.57; // 回転速度
float avoid_distance = 1.0;    // これ以上近いと障害物とみなす距離
float avoid_angle = 60;        // 障害物を探索する範囲

void scanCallback(sensor_msgs::LaserScan msg)
{
    int center_index = msg.ranges.size() / 2;                                   // ロボット前方方向を表すインデックス
    int index_avoid_angle = avoid_angle * (M_PI / 180.0) / msg.angle_increment; // 障害物を探索する範囲に相当するインデックス
    int start_index = center_index - index_avoid_angle / 2;
    int last_index = center_index + index_avoid_angle / 2;

    bool detect_obstacle = false;
    std::string detect_side;

    for (int i = start_index; i <= last_index; i++)
    {
        float range = msg.ranges[i]; // 距離

        // 値が無効な場合の排除
        if (range < msg.range_min || // エラー値の場合
            range > msg.range_max || // 測定範囲外の場合
            std::isnan(range))       // 無限遠の場合
            ;

        // 値が有効である場合
        else
        {
            if (range < avoid_distance) // 近くに障害物がある場合
            {
                detect_obstacle = true;
                if (i <= center_index)
                {
                    detect_side = "right";
                }
                else
                {
                    detect_side = "left";
                }
                break;
            }
        }
    }

    geometry_msgs::Twist cmd_vel;

    if (detect_obstacle)
    {
        if (detect_side == "right")
        {
            // その場で反時計回り回転
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = angular_velocity;
            pub.publish(cmd_vel); // publish
        }
        else
        {
            // その場時計回り回転
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = -angular_velocity;
            pub.publish(cmd_vel); // publish
        }
    }
    else
    {
        // 直進
        cmd_vel.linear.x = velocity;
        cmd_vel.angular.z = 0;
        pub.publish(cmd_vel); // publish
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "room_circuit_controller");
    ros::NodeHandle n;
    ros::NodeHandle pnh("~");

    pnh.getParam("velocity", velocity);
    pnh.getParam("angular_velocity", angular_velocity);
    pnh.getParam("avoid_distance", avoid_distance);
    pnh.getParam("avoid_angle", avoid_angle);

    pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    sub = n.subscribe("/scan", 1, scanCallback);

    ros::spin();
    return 0;
}
```

## 問題3.2
パラメータは一例です．

```xml
<?xml version="1.0"?>
<launch>
  <include file="$(find my_urdf_tutorial)/launch/robot_simulation.launch" />

  <node name="room_circuit_controller" pkg="room_circuit_controller" type="room_circuit_controller" output="screen">
    <param name="velocity" value="0.5" />
    <param name="angular_velocity" value="1.57" />
    <param name="avoid_distance" value="0.5" />
    <param name="avoid_angle" value="90" />
  </node>
</launch>
```