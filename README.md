# PAIA26 — 自然言語指示によるロボットアームの巡回動作

ROS 2 Humble／MoveIt 2／Ignition Gazebo上のCRANE+ V2を，ローカルLLMが解釈した日本語指示に従って動かすシミュレーション作品です．物体の検出や実際の点検は行わず，**指定地点への移動と腕の伸展・屈曲**を実行します．

## システム構成

```text
日本語の指示
    ↓
Ollama上のLLM（qwen3:4b）→ JSONによる巡回順序
    ↓
入力検証（LEFT / CENTER / RIGHT のみ許可）
    ↓
MoveIt 2：現在姿勢から軌道を計画・実行
    ↓
Gazebo：CRANE+ V2の複数関節を制御
    ↓
関節の実測値との比較・JSONLへの記録
```

LLMは**地点と巡回順序のみ**を出力します．任意の関節角度は生成・実行できず，各地点の動作はプログラムで定義しています．不明な指示や許可されない動作は拒否します．

## 動作内容

地点ごとに，アームを`extended`（伸展）→`retracted`（屈曲）の順に動かします．第1関節の目標角度は左`-0.40rad`，中央`0rad`，右`+0.40rad`です．第2〜第4関節にはCRANE+ V2のSRDFにある`vertical`／`home`姿勢を基準とした値を使用します．MoveIt 2が軌道を計画・実行し，その後の関節角度を計測します．

本作品は3地点の巡回順序を変えられます．最大5地点まで受け付け，LLMが返す地点名が許可集合外であれば実行しません．実際の障害物・作業対象の認識や，外部環境での安全性を保証するものではありません．

## 必要な環境

- ROS 2 Humble／Python 3／C++17／colcon
- MoveIt 2と`moveit_ros_planning_interface`
- CRANE+ V2のROS 2パッケージ（`crane_plus_description`，`crane_plus_moveit_config`，`crane_plus_gazebo`など）
- Ignition Gazebo（ROS 2と連携したシミュレータ，`move_group`，関節状態配信，アームコントローラが稼働すること）
- Ollama APIに到達できるネットワークと`qwen3:4b`モデル

第三者のライブラリ・モデル・ロボットパッケージは本リポジトリには同梱していません．[ATTRIBUTION.md](ATTRIBUTION.md)も参照してください．

## ビルドと実行

以下は**既にCRANE+ V2のシミュレーションが起動しているROS 2環境内**での実行例です．各ワークスペースのパスは実際のインストール先に読み替えてください．

```bash
source /opt/ros/humble/setup.bash
source /path/to/crane_plus_workspace/install/setup.bash

cd /path/to/paia-final-demo/moveit_ws
colcon build --packages-select paia26_moveit
source install/setup.bash
cd ..

export PAIA_OLLAMA_URL='http://127.0.0.1:11434'
export PAIA_OLLAMA_MODEL='qwen3:4b'

# AI推論のみ（ロボットは動かさない）
python3 moveit_demo.py --instruction '右、左、中央の順に巡回して'

# Gazeboで実際に動作（明示的に --execute を指定）
python3 moveit_demo.py --instruction '右、左、中央の順に巡回して' \
  --execute --log runs/patrol.jsonl
```

`PAIA_OLLAMA_URL`は**ROS 2を実行する環境からアクセス可能な**OllamaのURLを指定します．Ollamaサービスはインターネットへ公開しないでください．シミュレータを起動する方法はCRANE+ V2の公式ROS 2パッケージに従います．コース用DockerなどでGazeboのGUI起動に失敗する場合は，サーバーとGUIを分けて起動する方法もあります．

不適切な指示の例：

```bash
python3 moveit_demo.py --instruction '360度回転して' --log runs/rejected.jsonl
```

`--execute`を省略した実行はプラン表示のみです．未対応の指示についてはAIの回答と検証結果を確認してください．

## ディレクトリ

```text
final_demo.py                          LLM指示解釈，JSON検証（旧単関節版の基礎実装も含む）
moveit_demo.py                         最終デモ用のAI＋MoveIt実行・ログ記録
moveit_ws/src/paia26_moveit/          MoveItの複数関節制御用ROS 2パッケージ
tests/                                 ログ解析・指示検証などの単体テスト
README_MOVEIT.md                       MoveIt部分の補足
ATTRIBUTION.md                         利用した外部パッケージ・参考資料
```

`final_demo.py`の単関節デモは初期開発時の実装であり，**本作品の複数関節実行には`moveit_demo.py`を使用**します．

テストは`python3 -m unittest discover -s tests -q`で実行できます．実行ログは既定では`runs/`に記録され，GitHub公開対象から除外します．

## 確認できた結果（2026年10月26日）

`qwen3:4b`が「右、左、中央の順に巡回して」を`RIGHT → LEFT → CENTER`と解釈し，MoveIt 2で**3地点×2姿勢＝6動作**を完了しました．各地点について計画・実行の成功がログに記録されています．目標関節角度と動作完了後の観測値の最大絶対誤差は**0.00637rad**（約0.37度）でした．これは単一の実行試験の結果であり，総合的な精度評価ではありません．

## 制約と今後の課題

本作品はシミュレータ内で，指定地点の方向へ腕を動かすデモです．カメラによる物体認識・欠陥検出，物体の把持，実環境の障害物認識，実機での検証は実装していません．MoveItの動作計画機能を利用していますが，現実世界の衝突回避を保証しません．今後の発展として，視覚認識とタスク達成判定を追加できます．

## 課題としての工夫と出典

講義で扱ったROS 2／Gazeboの土台を利用しつつ，日本語の自由入力を構造化された巡回順序に変換する部分，許可された行動集合による制限，MoveItによる複数関節の軌道計画，関節の実測値に基づく検証と記録を組み合わせています．第三者のサンプルをそのまま提出したものではありません．公式サンプルとの関係や外部資料へのリンクは[ATTRIBUTION.md](ATTRIBUTION.md)に記載しています．公開サンプルの手順を参考にしつつ，最終課題では言語指示の解釈・動作選択の制限・実行結果の検証を追加した．

## ライセンス

本リポジトリで作成したプログラムと文書のライセンスは[Apache License 2.0](LICENSE)です．参照・利用した外部パッケージの著作権およびライセンスはそれぞれの配布元に従います．
