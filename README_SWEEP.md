# L字型ロボットアームの±90°巡回（最終版）

**本リポジトリの最終提出デモは`sweep_demo.py`と`paia26_sweep`です。** 旧版の`moveit_demo.py`（±0.4rad、地点ごとに伸展・屈曲）は開発履歴として保持しています。

## 目的と動作

ユーザーの日本語指示「中央、左、右、中央の順に巡回して」をOllama／`qwen3:4b`で解釈し、許可された順序と完全に一致する場合だけ次の6段階を実行します。

1. `CENTER_UP`：中央でCRANE+ V2の`vertical`姿勢（上向き、各関節0rad）にする。
2. `CENTER_FOREARM_LEVEL`：第2関節を上向きに維持し、第3関節を−π/2rad曲げ、**第3関節から先だけを水平**にする。
3. `LEFT`：L字姿勢を維持して第1関節を−π/2radへ。
4. `RIGHT`：L字姿勢を維持して第1関節を＋π/2radへ（左右180°差）。
5. `CENTER_RETURN`：L字姿勢で第1関節を0radへ戻す。
6. `HOME`：CRANE+ V2のSRDFで定義された`home`（q1=0, q2=−1.16, q3=−2.01, q4=−0.73rad）に戻す。

CRANE+ V2のURDFでは第3関節の軸が`0 −1 0`であり、`q3=−π/2`で先端側のリンクが横を向きます。これはグリッパ搭載のカメラが水平であることを意味しません。今回のモデルでは**グリッパのカメラも画像認識も使用していません**。

## 動作前の計画・安全側の制限

- PythonはLLM出力を構造化して、`['CENTER','LEFT','RIGHT','CENTER']`以外を拒否します。LLMによる自由な角度・姿勢生成は行いません。
- 開始姿勢は既知の`home`または`vertical`付近に限定します。必要なら「事前のhome復帰」も含め、全区間のMoveIt事前計画を完了してから実行します。
- 各段階で現在の関節状態から再計画・実行し、4関節の目標と実測との差を検査します。閾値は最大絶対誤差0.07radです。失敗すれば後続を中断します。
- **`--execute`を付けない限り、ロボットを動かしません。**

## 実行例

```bash
# ROS 2 Humble、CRANE+ V2、MoveIt 2のワークスペースを読み込んだ環境内
cd /path/to/paia26-final-demo/moveit_ws
colcon build --packages-select paia26_sweep
source install/setup.bash
cd ..

export PAIA_OLLAMA_URL='http://127.0.0.1:11434'  # 実際の接続先に合わせる
export PAIA_OLLAMA_MODEL='qwen3:4b'
python3 sweep_demo.py --instruction '中央、左、右、中央の順に巡回して'
python3 sweep_demo.py --instruction '中央、左、右、中央の順に巡回して' --execute \
  --log runs/sweep.jsonl
```

前提となるGazebo、`move_group`、関節コントローラ、`/joint_states`、`/clock`は起動済みである必要があります。パスは環境に合わせて変更してください。

## 2026年10月29日の確認結果

LLMの順序解釈、MoveItでの全6段階の事前計画、実行、4関節の到達確認に成功しました。左の実測値−1.56499rad、右＋1.56525rad、左右差**179.35°**。6段階の最大誤差は**0.00708rad**。Python単体テスト22件がPASS。いずれも単一のシミュレーション実験の結果です。

## 制約・第三者資産

ロボットは定義済みの方向に腕を向けるだけで、周囲の対象や欠陥を撮像・検出・点検していません。物体の把持、実機での検証、計画シーン外の障害物に対する安全保証もありません。

CRANE+ V2のURDF、SRDF、MoveItのサンプル、ROS2の構成方法はRT Corporationの[公式パッケージ](https://github.com/rt-net/crane_plus)を参照しています。詳細は[ATTRIBUTION.md](ATTRIBUTION.md)に記載しています。
