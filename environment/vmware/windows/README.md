# WindowsにVMware WorkStation Player 16 をインストール 
[前のページ](../../)

## 概要

Windows 10のPCにVMWare WorkStation Player 16をインストールします．

途中手順通りにいかない場合があります．その際はトラブルシューティングの章を参考にしてください．

また，PCのストレージは20GBほど空きがあることが望ましいです．



## 手順

### インストーラのダウンロード

以下のサイトにアクセスします．

https://www.vmware.com/jp/products/workstation-player/workstation-player-evaluation.html

少し下にスクロールし，「VMWare Workstation 16 Player for Windows の試用」の今すぐダウンロードをクリックします．するとダウンロードが始まるので，好きな場所に保存してください．

<img src='https://raw.githubusercontent.com/Decwest/introduction-to-ros/main/environment/vmware/windows/fig/JCwr3rH.png' width="500" >


### インストーラの実行
インストーラをダブルクリックしてしばらくすると，以下の画面が立ち上がります．（立ち上がらない場合はトラブルシューティングの章を参考にしてください）

<img src='https://raw.githubusercontent.com/Decwest/introduction-to-ros/main/environment/vmware/windows/fig/s9UdVzJ.png' width="500" >


次へをクリックします．

<img src='https://raw.githubusercontent.com/Decwest/introduction-to-ros/main/environment/vmware/windows/fig/kPYozFH.png' width="500" >


問題なければ同意してください．次へ．

<img src='https://www.kkaneko.jp/tools/vmware/368.png' width="500" >


ここで，人によっては「互換性のあるセットアップ」の画面が出ることがあります．（筆者のノートPCでは出ました）

WHPの自動インストールを選択しましょう．次へ．

参考：https://www.kkaneko.jp/tools/vmware/vmwareplayerwin.html

<img src='https://raw.githubusercontent.com/Decwest/introduction-to-ros/main/environment/vmware/windows/fig/G18VX8d.png' width="500" >


拡張キーボードドライバはオフでよいです．

次へ．

<img src='https://raw.githubusercontent.com/Decwest/introduction-to-ros/main/environment/vmware/windows/fig/RW9ApYA.png' width="500" >


オフでもよいですが，VMwareに貢献したい方はオンにしましょう．

次へ．

<img src='https://raw.githubusercontent.com/Decwest/introduction-to-ros/main/environment/vmware/windows/fig/2F2MaCM.png' width="500" >


ショートカットを作成したい方はチェックを入れましょう．

個人的にはチェックしたままでよいと思います．

次へ．

<img src='https://raw.githubusercontent.com/Decwest/introduction-to-ros/main/environment/vmware/windows/fig/b9H4EdF.png' width="500" >


インストールを開始しましょう．

<img src='https://raw.githubusercontent.com/Decwest/introduction-to-ros/main/environment/vmware/windows/fig/VjtmqGn.png' width="500" >


正常にインストールが終わるとこのような画面になります．

完了を押して終了します．

### 起動

<img src='https://raw.githubusercontent.com/Decwest/introduction-to-ros/main/environment/vmware/windows/fig/4MNxNiq.png' width="100" >


デスクトップ等に生成されたショートカットから起動しましょう．

<img src='https://raw.githubusercontent.com/Decwest/introduction-to-ros/main/environment/vmware/windows/fig/ZNLi5iV.png' width="500" >


非営利目的で使用するにチェックして，続行をクリックしましょう．

<img src='https://raw.githubusercontent.com/Decwest/introduction-to-ros/main/environment/vmware/windows/fig/OlP3yqS.png' width="500" >


完了をクリックしましょう．

<img src='https://raw.githubusercontent.com/Decwest/introduction-to-ros/main/environment/vmware/windows/fig/ThtpoCk.png' width="500" >



起動すればOKです．


## トラブルシューティング
- インストーラが起動できない
インストーラを実行すると再起動を求められることがありました．（画像が無くて申し訳ありません）
この時は，素直に再起動をすることでインストーラが実行できるようになりました．
おそらく，[この現象](https://docs.vmware.com/jp/VMware-Tools/11.3.0/com.vmware.vsphere.vmwaretools.doc/GUID-737341FF-1AD7-4006-B904-8867FB557147.html)です．
再起動しても直らない場合は，[このリンク先](https://support.microsoft.com/ja-jp/topic/%E6%9C%80%E6%96%B0%E3%81%AE%E3%82%B5%E3%83%9D%E3%83%BC%E3%83%88%E3%81%95%E3%82%8C%E3%82%8B-visual-c-%E3%81%AE%E3%83%80%E3%82%A6%E3%83%B3%E3%83%AD%E3%83%BC%E3%83%89-2647da03-1eea-4433-9aff-95f26a218cc0)からMicrosoft Visual C++ 再頒布可能パッケージを手動でインストールしてみてください．
32bit PCの場合x86，64bit PCの場合x64をインストールしましょう．

## リンク
[次のページ](../install-ubuntu/)

[目次](../../../)