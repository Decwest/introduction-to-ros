# 演習：部屋を巡回するルンバのROSプログラミング

[前のページ](../)

## 概要

今までの章でROSの基礎，2D LiDARを積んだ移動ロボットのシミュレーションモデルの作成を行いました．本章では，2D LiDARで取得した周囲の障害物情報を用いて，障害物にぶつからないように部屋を巡回する知能ロボットのコード作成をROSを用いて行います．

## 問題設定確認
1. 2D LiDARの情報を取得
1. ロボットに速度指令値を与える

[URDFにアクチュエータを追加](../gazebo/actuator/)と[ロボットモデルにセンサを追加](../gazebo/sensor/)で設定した通り，2D LiDARのROSトピック名は`/scan`, 速度指令値のROSトピック名は`/cmd_vel`です．

ここで，これらのトピックの型を調べてみましょう．

```bash
roslaunch my_urdf_tutorial robot_simulation.launch
```

---

問題0：[CommandLine Tools](../ros/command/)の内容を思い出しながら，/scan, /cmd_velの型を調べましょう．

---

解答：`rostopic info /scan`等で以下のようにわかります．

- /scan: sensor_msgs/LaserScan
- /cmd_vel: geometry_msgs/Twist

よって，以下では
- sensor_msgs/LaserScan型のROSトピック/scanをサブスクライブ
- geometry_msgs/Twist型のROSトピック/cmd_velをパブリッシュ

するROSノードを作成しましょう．

## 問題1：上記のpubsubを行うROSノードを作成する
問題1.1: 新たに`room_circuit_controller`という名前のROSパッケージを作成しましょう．

問題1.2: room_circuit_controller.cppを適切な場所に作成し，上記のROSノードを作成しましょう．

設定
- サブスクライブのコールバック関数名は`scanCallback`
- ノード名は`room_circuit_controller`
- コールバック関数の中身はまだ記述しなくて大丈夫です．また，サブスクライブした情報をもとにパブリッシュするため，サブスクライバのコールバック関数内にパブリッシュの記述を書きます．よって，main関数には`while(ros::ok())`ではなく`ros::spin()`を書けばよいです．

参考：[トピック通信実践](../ros/topic/)

解答は[こちら](./answer1/)

## 問題2：/scanの内容に応じて/cmd_velをpublish
/scan, /cmd_velの具体的な内容は以下のリンク先記載の通りです．
ROSトピックの型名でググると出てくるので，ググれるようになりましょう．

[/scan](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html)

[/cmd_vel](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html)

今回は，以下のような動作フローを考えてみます．
- 2D LiDARの障害物情報をロボット左側から右側に向けて見る
    - 以下の図のようにavoid_angle度分探索する．なお，この領域はロボット前方を軸に左右対称．
- 障害物までの距離がavoid_distanceより小さい場合「障害物がある」とする
    - 前方より左側に「障害物があったら」，速度angular_velocityでその場で時計回り回転
    - 前方より左側にはないが右側に「障害物があったら」,
    速度angular_velocityでその場で反時計回り回転
    - 前方より左側にも右側にもない場合は速度velocityで直進

<img src='./fig/1.png' width="500" >

問題2.1: 以下のソースコードにおける「ここを書く！！！！！」の部分を適切に埋めましょう．

```cpp
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher pub;
ros::Subscriber sub;

// variables
float velocity = 0.2; // 直進速度
float angular_velocity = 1.57; // 回転速度
float avoid_distance = 1.0; // これ以上近いと障害物とみなす距離
float avoid_angle = 60; // 障害物を探索する範囲

void scanCallback(sensor_msgs::LaserScan msg)
{
    int center_index = msg.ranges.size() / 2;             // ロボット前方方向を表すインデックス
    int index_avoid_angle = avoid_angle * (M_PI / 180.0) / msg.angle_increment; // 障害物を探索する範囲に相当するインデックス
    int start_index = center_index - index_avoid_angle / 2; // 探索開始インデックス
    int last_index = center_index + index_avoid_angle / 2; // 探索終了インデックス

    bool detect_obstacle = false;
    std::string detect_side;

    for (int i = start_index; i <= last_index; i++)
    {
        float range = ここを書く！！！！！; // 距離

        // 値が無効な場合の排除
        if (range < msg.range_min || // エラー値の場合
            range > msg.range_max || // 測定範囲外の場合
            std::isnan(range))       // 無限遠の場合
            ;

        // 値が有効である場合
        else
        {
            if (ここを書く！！！！！) // 近くに障害物がある場合
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
            ここを書く！！！！！
        }
    }
    else
    {
        // 直進
        ここを書く！！！！！
    }
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

ヒント：cmd_vel.linear.zに直進速度，cmd_vel.angular.zに回転速度を入れます．

問題2.2: CMakeLists.txtを適切に編集しましょう．

問題2.3: ビルドしましょう．

問題2.4: my_urdf_tutorial/launch/robot_simulation.launchをincludeし，かつroom_circuit_controllerノードを実行するroom_circuit_controller.launchを適切な場所に作成しましょう．


参考

[トピック通信実践](../ros/topic/)

[roslaunch](../ros/roslaunch/)

解答は[こちら](./answer2/)

### 動作確認

```bash
roslaunch room_circuit_controller room_circuit_controller.launch
```

動いたらOKで，さらに前方に障害物を置いてみて障害物回避が行われれば成功です！

また，実際に部屋のような環境で確認したい方は，以下の手順に従うと部屋のような環境を用意できます！
1. my_urdf_tutorial配下に`worlds`ディレクトリを作成
1. worldディレクトリの中に`room.world`という名前のファイルを作成
1. `room.world`に[こちらの内容](https://raw.githubusercontent.com/Decwest/introduction-to-ros/main/world/room.world)をコピペ

## 問題3
問題3.1: velocity, avoid_velocity, avoid_distance, avoid_angleをrosparam化しましょう．

問題3.2: launchファイルからrosparamを渡し，変更できるようにして見ましょう．

## リンク

[目次](../../)


---

## 余談
### 今回扱えなかったが非常に重要なもの
- カスタムROSメッセージ
- TF
