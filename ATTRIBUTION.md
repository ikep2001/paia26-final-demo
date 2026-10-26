# 参照した既存技術・資料

この作品で使用したロボットとシミュレーション環境は既存の外部パッケージに依存しています．**外部パッケージ一式を本リポジトリに含めていません**．一部のプログラム構成は公開サンプルを参考にしており，下記に出典を記載しています．使用時には各プロジェクトのライセンスと配布条件に従ってください．

- [CRANE+ V2 ROS 2パッケージ（RT Corporation）](https://github.com/rt-net/crane_plus)：Gazebo連携とMoveIt設定，`pose_groupstate`等のサンプルを参照．`home`／`vertical`姿勢の関節値は同リポジトリの`crane_plus_moveit_config/config/crane_plus.srdf`に基づく．
- [MoveIt 2](https://moveit.picknik.ai/)：ロボットアームの動作計画と軌道実行．
- [ROS 2 Humble](https://docs.ros.org/en/humble/)：ROS 2の通信・パッケージ環境．
- [Gazebo](https://gazebosim.org/)：物理シミュレーション．
- [Ollama](https://ollama.com/)：ローカルLLMの推論API．利用するモデルの配布条件も確認してください．

## 公式サンプルとの関係

- `station_move.cpp`は，RT Corporationの`crane_plus_examples/src/pose_groupstate.cpp`等で示されるMoveItの使用方法を参考にし，固定地点ごとの目標姿勢・軌道計画・実行後の観測誤差の検査を独自に実装した．
- `station.launch.py`は，`crane_plus_examples/launch/example.launch.py`のロボット記述・運動学パラメータ設定方法を参考にした．
- `final_demo.py`と`moveit_demo.py`には，日本語の指示からJSONの地点列を得て検証し，実行を制御して結果を記録する処理を実装した．

CRANE+ V2パッケージの権利表示およびライセンスは配布元に従う．本リポジトリに含めないROS 2／MoveIt／Gazebo／Ollama等の配布物はそれぞれのライセンスに従う．

本リポジトリは`LICENSE`に掲げるApache License, Version 2.0を採用する．これは第三者パッケージのライセンスを変更するものではない．
