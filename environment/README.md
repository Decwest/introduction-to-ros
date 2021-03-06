# Ubuntu環境の用意

[前のページ](../)

[目次](../)

## 概要
ROSを動かすには[Ubuntu](http://www.ubuntulinux.jp/ubuntu)というOSが必要です．

そこで，ここではWindowsやMacユーザーがUbuntuを実行できる環境を用意する方法を紹介します．

様々な方法がありますが，ここでは導入の簡潔さを重視し，[VMware](https://www.vmware.com/jp/products/workstation-player.html)を用いてPC上に仮想マシンを構築し，そこにUbuntuをインストールする方法をとります．

## VMwareのインストール

[Windowsの方](./vmware/windows/)

[Macの方](./vmware/mac)

## VMware上でのUbuntu 20.04のインストール
[Windowsの方](./vmware/install-ubuntu/windows)

[Macの方](./vmware/install-ubuntu/mac)


## リンク
[次のページ](./ros/)

[目次](../)

---

### 余談
その他には以下を用いて構築することを検討していました．
- Virtual box
    
    VMwareのほうが軽い印象があったので（要検証），VMがダメだった人にはVirtual boxを薦めようと思っていました．
- Docker
    
    軽いしDockerfile配れるし当初は最有力候補でしたが，X windowの設定が初学者には大変かと思いやめました．
    
    [Tiryoh](https://github.com/Tiryoh)さんの[Webブラウザで試せるDockerfile](https://memoteki.net/archives/2955)を用いればGUIの問題は解消し非常に簡潔にROS環境を導入できますが，Ubuntu環境の用意とROSのインストールも込みで扱おうと思ったため今回は使用しませんでした．ROS環境をサクッと用意して試したい方には大変おすすめです．
- WSL2
    
    WindowsユーザーはWSL2でもよいかなと思いましたが，X Windowの設定とか大変なのでやめました．VMより軽いので興味のある方は入れてみてください．