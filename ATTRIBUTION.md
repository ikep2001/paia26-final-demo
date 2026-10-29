# 外部技術・参考資料／ライセンス

本作品は、ローカルLLMの指示解釈をROS 2／MoveIt 2／Gazeboの既存スタックに接続する独自デモです。以下の**外部プログラム・重み・ロボットモデルは本リポジトリに同梱していません**。各配布元のライセンス・利用条件が適用されます。

- [RT Corporation: CRANE+ V2 ROS 2 packages](https://github.com/rt-net/crane_plus)（`humble`ブランチ）。ロボットモデル、Gazebo・MoveIt設定、基準姿勢を利用。`crane_plus_moveit_config/config/crane_plus.srdf`の`vertical`／`home`姿勢と、`crane_plus_description/urdf/crane_plus.urdf.xacro`および`crane_plus.xacro`の関節軸・制限を参照。
- [CRANE+ V2 のMoveIt実行例](https://github.com/rt-net/crane_plus/tree/humble/crane_plus_examples)：`pose_groupstate.cpp`、`example.launch.py`のMoveGroupインターフェース、ロボット記述、運動学設定の組み方を参考にした。
- [MoveIt 2](https://moveit.picknik.ai/)：ROS2上の計画、計画結果、軌道実行、関節状態照会。
- [ROS 2 Humble](https://docs.ros.org/en/humble/)：ROS2ノードと制御通信。
- [Gazebo](https://gazebosim.org/)：シミュレーション実行環境。
- [Ollama](https://ollama.com/)：ローカルLLM推論API。`qwen3:4b`のモデル利用は配布元の条件に従う。

## 本リポジトリの追加実装

- `sweep_demo.py`：日本語入力→JSONの巡回順序を受け取って許可済みの`CENTER → LEFT → RIGHT → CENTER`と照合。エラー時は動作しない。計画・実測値をJSONLで記録。
- `moveit_ws/src/paia26_sweep/src/sweep_move.cpp`：`home`開始判定と必要時の準備経路、全6段階の事前計画、L字姿勢（`q3=-π/2`）での±90°旋回、各動作の関節到達確認を実装。
- `moveit_ws/src/paia26_sweep/launch/sweep.launch.py`：CRANE+ V2の記述・SRDF・運動学パラメータを移植せず外部パッケージから読み込む方式。公式サンプルの構成を参考にした。
- 旧版`station_move.cpp`／`station.launch.py`はMoveItの初期段階のデモで、出典の記載は引き続き適用される。

注意：シミュレータ上で計画・実行できることは、実機の衝突安全性や実物体の点検の成立を意味しません。カメラによる視覚認識や物体把持は未実装です。

このリポジトリの独自プログラムと文書は[Apache License 2.0](LICENSE)に従います。外部パッケージの著作権・ライセンスは変更されません。
