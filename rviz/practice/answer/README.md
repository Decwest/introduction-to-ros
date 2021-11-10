# ルンバのモデル作成：解答例

### リンク作成
1. 円柱でルンバの車体body_linkを作成する
    - 半径150mm, 高さ72mmの円柱body_link
    ```xml
    ```
1. 球でルンバのキャスターball_linkを作成する
    - 半径10mmのキャスターball_link_front, ball_link_back
    ```xml
    ```
1. 円柱でルンバの車輪wheel_linkを作成する
    - 半径36mm, 高さ16mmの円柱wheel_link_right, wheel_link_left
    ```xml
    ```
### ジョイント作成
1. base_linkとbody_linkの固定ジョイントbody_jointを作成する
    ```xml
    ```
1. body_linkとball_linkの固定ジョイントball_joint_front, ball_joint_backを作成する
    ```xml
    ```
1. body_linkとwheel_linkの回転ジョイントwheel_joint_right, wheel_joint_leftを作成する
    ```xml
    ```

全体の解答

    ```xml
    ```
