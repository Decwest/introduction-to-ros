# 演習：ルンバのモデル作成

[前のページ](../urdf/)



## 概要

前ページで学んだことを活かして，ルンバのモデルを作成しましょう．

## ルンバの寸法
単位はmmです．
<img src='./fig/1.jpg'>

## 演習
以下の手順に沿ってルンバのモデルを作成しましょう．

roomba.urdfに記述していきます．

### リンク作成
1. base_linkを作成する
1. 円柱でルンバの車体body_linkを作成する
    - 半径150mm, 高さ72mmの円柱body_link
1. 球でルンバのキャスターball_linkを作成する
    - 半径10mmのキャスターball_link_front, ball_link_back
1. 円柱でルンバの車輪wheel_linkを作成する
    - 半径36mm, 高さ16mmの円柱wheel_link_right, wheel_link_left
### ジョイント作成
1. base_linkとbody_linkの固定ジョイントbody_jointを作成する
1. body_linkとball_linkの固定ジョイントball_joint_front, ball_joint_backを作成する
1. body_linkとwheel_linkの回転ジョイントwheel_joint_right, wheel_joint_leftを作成する

解答は[こちら](./answer/)

## リンク

[次のページ](../../gazebo/)

[目次](../../)


---

## 余談
### xacro