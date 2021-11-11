# URDFに物理量を追加

[前のページ](../about/)

## 概要

前章で作成したroomba.urdfに以下の物理量を追加します．
- リンク
    - 衝突判定に使う形状
    - 慣性モーメント
    - 色（RvizとGazeboで色の表現方法が異なるため）
    - （地面に接地するリンク）摩擦
- （回転）ジョイント
    - 出力最大トルク，速度
    - 減衰係数，摩擦

my_urdf_tutorial/urdfの中に，roomba_sim.urdfというroomba.urdfをコピペしたファイルを用意し，追記していきます．

参考：[Gazebo Tutorial](http://gazebosim.org/tutorials/?tut=ros_urdf)


## リンク
例として，wheel_link_rightに物理量を追記していきます．

全体像
```xml

```

### 衝突判定に使う形状
#### `<collision>`タグ

```xml
<collision>
    <geometry>
        <cylinder radius="0.036" length="0.016"/>
    </geometry>
    <origin xyz="0 0 0" rpy="0 0 0"/>
</collision>
```
- collision：衝突判定に関する記述
    - geometry：基本図形の形状
    - origin：リンク座標系から見たリンクの重心の位置

urdfのvisualと全く同じ文法なので，詳細は割愛します．

visualとcollisionは基本的には同じ寸法でよいですが，見た目より衝突判定を緩くしたい場合はcollisionの寸法を少し小さく書いたりします．

今回は同じ寸法で記述します．

### 慣性モーメント

```xml
    <inertial>
        <origin xyz="0 0 0" rpy="0 0 0" />
        <mass value="0.10" />
        <inertia ixx="${wheel_mass*(3*wheel_radius*wheel_radius+wheel_height*wheel_height)/12}" 
            ixy="0"  ixz="0"
            iyx="0"  iyy="${wheel_mass*(3*wheel_radius*wheel_radius+wheel_height*wheel_height)/12}" 
            iyz="0"
            izx="0"  izy="0"  izz="${wheel_mass*(wheel_radius*wheel_radius)/2}" />
    </inertial>
```

- inertial：質量と慣性を定義
    - origin：リンク座標系から見た重心位置
    - mass：リンクの質量
    - inertia：リンクの重心周りの慣性モーメント

originについては0でよいでしょう．

massについては，今回は車輪の質量を0.1kgとします．

問題はinertiaで，これは慣性モーメントテンソルを表しています．ここは剛体の物理学を用いて計算します．詳細は余談をご覧ください．
今回は密度は一様であると仮定して計算します．

<img src='./fig/1.png' width="500" >

今回は車輪に対して上図のようにxyz軸が重心に位置しています．赤がx, 緑がy, 青がzです．(RGBとxyzの順番が一致していると覚えましょう)

よって，xyz周りの慣性モーメントはそれぞれ以下の式で求められます．なお，$M$は質量，$R$は半径，$h$は高さです.


- x軸周り

    $I_{xx}=M\left(\dfrac{R^2}{4}+ \dfrac{h^2}{12} \right)$

- y軸周り

    $I_{yy}=M\left(\dfrac{R^2}{4}+ \dfrac{h^2}{12} \right)$
    
- z軸周り

    $I_{zz}=\dfrac{1}{2}MR^2$
    

参考：[剛体の慣性モーメントの計算](https://physics-school.com/moment-of-inertia/)

今回，$M=0.1$, $R=0.036$, $h=0.016$なので，上式に代入することでinertiaの項目を埋めることができます.

### 色
gazebo記述する

### 摩擦
gazebo記述する

## ジョイント
例として，wheel_joint_rightに物理量を追記していきます．

全体像

### 出力最大トルク，速度
limit

### 減衰係数，摩擦
damping, mu

以上の物理量を他のリンク，ジョイントにも追加すると以下のようになります．


## リンク

[次のページ](../urdf/)

[目次](../../)



---

## 余談
### 慣性モーメントテンソルについて
慣性モーメントは回転軸に対する物体の回転のしにくさを表しますが，xyzの各座標軸における回転のしにくさを以下のようにひとつの行列で表したものを慣性モーメントテンソルといいます．
このうち，$I_{xx}$はx軸周りの慣性モーメント，$I_{yy}$はy軸周りの慣性モーメント，$I_{zz}$はz軸周りの慣性モーメントになります．非対角成分は慣性乗積といいます．

$I=\left(\begin{array}{c}I_{xx} & I_{xy} & I_{xz}\\ I_{yx} & I_{yy} & I_{yz}\\ I_{zx} & I_{zy} & I_{zz}\end{array}\right)$

ここで，xyz座標軸をうまくとることで，慣性乗積を0にすることができ，以下の形で表すことができます．イメージ的には，例えば今回の車輪のように，xyz軸それぞれに対して物体の形状が対称である場合，各軸に対して"きれいに"回転するので慣性乗積が0になります．（対称な場合，非対角成分が回転の各ステップにおいて相殺されるイメージです）

$I=\left(\begin{array}{c}I_{xx} & 0 & 0\\ 0 & I_{yy} & 0\\ 0 & 0 & I_{zz}\end{array}\right)$

参考リンク
- [剛体の慣性モーメントの計算](https://physics-school.com/moment-of-inertia/)
    - 円柱の各xyz軸における慣性モーメントの求め方や，慣性モーメントテンソルについても記載されています．
- [20分で分かる慣性テンソル【古典力学_回転の力学】](https://youtu.be/CVUIrfvHemU)
    - 慣性モーメントテンソルの導出からその物理的意味まで分かりやすく説明されています．