# VMware上でのUbuntu 20.04のインストール
[前のページ](../../)

## 概要
VMwareによる仮想マシンにUbuntu 20.04をインストールします．VMwareのインストールが済んでいない方は以前のページをご覧ください．

## 手順
### Ubuntu20.04 isoイメージのダウンロード
以下のサイトにアクセスし，Ubuntu 20.04のisoイメージ（インストーラ）をダウンロードします．
https://jp.ubuntu.com/download

<img src='https://raw.githubusercontent.com/Decwest/introduction-to-ros/main/environment/vmware/install-ubuntu/fig/2.png' width="500" >


### VMwareにインストール
VMwareを開き，新規仮想マシンの作成を選択．

<img src='https://raw.githubusercontent.com/Decwest/introduction-to-ros/main/environment/vmware/install-ubuntu/fig/1.png' width="500" >

インストーラ　ディスクイメージファイルを選択し．参照から先程ダウンロードしたisoイメージを選択します．
次へ．

<img src='https://raw.githubusercontent.com/Decwest/introduction-to-ros/main/environment/vmware/install-ubuntu/fig/3.png' width="500" >


ログイン情報等を入れます．いずれも英数字を入力します．
- フルネーム：ログイン画面で表示される名前です．
- ユーザー名：端末 (Terminal) と呼ばれる画面に表示される名前です．
小文字、0 ～ 9 の数字、ダッシュのみを使用できます．
- パスワード

<img src='https://raw.githubusercontent.com/Decwest/introduction-to-ros/main/environment/vmware/install-ubuntu/fig/4.png' width="500" >

仮想マシン名は何でもよいですが，ここではバージョン名をわかりやすくするために，Ubuntu 20.04としましょう．
場所はそのままで大丈夫です．

<img src='https://raw.githubusercontent.com/Decwest/introduction-to-ros/main/environment/vmware/install-ubuntu/fig/5.png' width="500" >

ディスク容量とは，仮想マシンのストレージ（どのくらいデータを詰め込めるか）の値です．基本的にデフォルトの20GBで問題ないです．
仮想ディスクを複数に分割を選択します．

<img src='https://raw.githubusercontent.com/Decwest/introduction-to-ros/main/environment/vmware/install-ubuntu/fig/6.png' width="500" >

最後に確認画面が出ます．
ここで，少し設定をいじります．
ハードウェアをカスタマイズをクリックしましょう．

<img src='https://raw.githubusercontent.com/Decwest/introduction-to-ros/main/environment/vmware/install-ubuntu/fig/7.png' width="500" >

ここで以下のようにメモリを4096MB，プロセッサ数を4に変更しましょう．
今回の講習ではGazeboという物理シミュレータを利用するため，少しパソコンのスペックを増強します．
なお，もともとメモリが4096MBとなっている場合もあります．その際はプロセッサ数だけ変更すればよいです．
変更したら閉じるを押して閉じましょう．

<img src='https://raw.githubusercontent.com/Decwest/introduction-to-ros/main/environment/vmware/install-ubuntu/fig/8.png' width="500" >


<img src='https://raw.githubusercontent.com/Decwest/introduction-to-ros/main/environment/vmware/install-ubuntu/fig/9.png' width="500" >


変更が反映されていることが確認出来たら，完了を押しましょう．インストールが開始します．

<img src='https://raw.githubusercontent.com/Decwest/introduction-to-ros/main/environment/vmware/install-ubuntu/fig/10.png' width="500" >

OKを押してください．

<img src='https://raw.githubusercontent.com/Decwest/introduction-to-ros/main/environment/vmware/install-ubuntu/fig/11.png' width="500" >

ダウンロードしてインストールを選択してください．
管理者権限が求められたらはいを選択してください．

<img src='https://raw.githubusercontent.com/Decwest/introduction-to-ros/main/environment/vmware/install-ubuntu/fig/12.png' width="500" >

10分ほど待つとインストールが完了します．


Ubuntuが立ち上がる前にこのような画面になるかもしれませんが，待機していれば大丈夫です．

<img src='https://raw.githubusercontent.com/Decwest/introduction-to-ros/main/environment/vmware/install-ubuntu/fig/13.png' width="500" >

立ち上がりました！ログインしてみましょう．

<img src='https://raw.githubusercontent.com/Decwest/introduction-to-ros/main/environment/vmware/install-ubuntu/fig/14.png' width="500" >

雲のマークのセットアップは基本的にスキップで大丈夫です．

<img src='https://raw.githubusercontent.com/Decwest/introduction-to-ros/main/environment/vmware/install-ubuntu/fig/15.png' width="500" >

Ubuntuのデスクトップが立ち上がりました．
それでは，次のページでこのUbuntuにROSをインストールします．

## リンク
[次のページ]()

[目次](../../..//