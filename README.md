# CC1120-EM
[CC1120のやつ](https://github.com/yui-project/CC1120/tree/EM)を扱いやすくしたもの<br>
ようはレジスタ設定値を変更しやすくしたもの<br>
とはいえほとんど変えてない<br>
CW（ASK/OOK）のレジスタ値を変えたぐらい

<ul>
<li>setup内でCC1120.CWbegin()を実行すれば変調方式がCWに、CC1120.FSKbegin()ならFSK。</li>
<li>loop内のCC1120.setFREQ()で周波数の設定</li>
<li>CC1120.setPWRで出力強度の設定</li>
<ul>
<li>現在、フロントエンドの不調により想定通りの動作をしてくれない</li>
</ul>
</ul>