# roslaunch

[前のページ](../rosparam)

[目次](../../)



## 概要



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

### 名前空間について

### rosrunオプション
https://qiita.com/srs/items/0a6c7dfc2520833d6958