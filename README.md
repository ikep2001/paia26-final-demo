# PAIA26 — 日本語指示で動くL字型ロボットアームの左右180°巡回

**最終版：L字姿勢で左右±90°を見渡すCRANE+ V2シミュレーション（2026-10-29検証）**

日本語の巡回指示をOllamaのローカルLLM（`qwen3:4b`）で解釈し、ROS 2 Humble／MoveIt 2／Ignition Gazebo上のCRANE+ V2で、あらかじめ定義した一連の姿勢を計画・実行するデモです。**実機、画像認識、対象物の点検、物体把持は対象外**です。

## 今回の最終デモ

入力：**「中央、左、右、中央の順に巡回して」**

LLMのJSON出力から得られた巡回順序 `CENTER → LEFT → RIGHT → CENTER` を検証し、この順序が許可済みの実演パターンと一致する場合だけ次の6段階を実行します。LLMが動作角度を生成したり、任意の関節指令を実行したりすることはありません。

| 段階 | 動作 | 第1関節 q1 | 第2～第4関節 (q2, q3, q4) |
|---|---|---:|---|
| 1 `CENTER_UP` | 中央で上向きに伸展 | 0° | (0°, 0°, 0°) |
| 2 `CENTER_FOREARM_LEVEL` | 第3関節から先を水平にするL字姿勢 | 0° | (0°, −90°, 0°) |
| 3 `LEFT` | L字姿勢のまま左を向く | −90° | (0°, −90°, 0°) |
| 4 `RIGHT` | L字姿勢のまま右を向く | ＋90° | (0°, −90°, 0°) |
| 5 `CENTER_RETURN` | L字姿勢で中央へ戻る | 0° | (0°, −90°, 0°) |
| 6 `HOME` | 既定の折り畳み姿勢へ戻る | 0° | (−1.16, −2.01, −0.73) rad |

**ここで「水平」なのは腕全体ではなく、第3関節から先（前腕・グリッパ側）のリンク**です。カメラを水平に保つという意味ではありません。実行対象にはグリッパ搭載カメラはありません。

## システム構成

```text
日本語の指示
  → Ollama / qwen3:4b（JSONによる巡回順序の解釈）
  → Pythonの許可順序検証（CENTER → LEFT → RIGHT → CENTER）
  → MoveIt 2（開始姿勢・6遷移を事前に軌道計画）
  → ROS2コントローラ・Gazebo上のCRANE+ V2
  → MoveItの実行結果＋実測関節角度の確認
  → JSONL記録
```

開始姿勢がCRANE+ V2の既知の`home`／`vertical`に近い場合だけ処理し、必要なら`home`に戻す軌道も事前計画します。全区間の事前計画が成功してから、各区間を実行直前に再計画して送信します。失敗した場合は後続動作を停止し、成功を報告しません。Gazeboの計画シーンに含まれていない障害物への安全性や実機の安全性を保証するものではありません。

## 実行環境と再現手順

- Ubuntu／ROS 2 Humble、Python 3、C++17、colcon、MoveIt 2
- [CRANE+ V2 ROS 2 パッケージ](https://github.com/rt-net/crane_plus)およびGazebo（`move_group`、アームコントローラ、`/joint_states`、`/clock`が有効）
- Ollama APIと`qwen3:4b`モデル。外部パッケージやLLMの重みはこのリポジトリに同梱しません。

```bash
# 既存のCRANE+ V2 / Gazebo / move_groupを起動済みのROS 2環境で
source /opt/ros/humble/setup.bash
source /path/to/crane_plus_workspace/install/setup.bash
cd /path/to/paia26-final-demo/moveit_ws
colcon build --packages-select paia26_sweep
source install/setup.bash
cd ..

export PAIA_OLLAMA_URL='http://127.0.0.1:11434'  # 環境に合わせて変更
export PAIA_OLLAMA_MODEL='qwen3:4b'

# 推論とMoveItの事前計画のみ（ロボットは動かない）
python3 sweep_demo.py --instruction '中央、左、右、中央の順に巡回して'

# シミュレータ上で動作を実行
python3 sweep_demo.py --instruction '中央、左、右、中央の順に巡回して' \
  --execute --log runs/sweep.jsonl

# Python単体テスト
python3 -m unittest discover -s tests -q
```

注意：ROS2ワークスペースのパスは各自の環境に読み替えてください。ROS2環境は`source`の際に未定義環境変数を参照する場合があるため、シェルで`set -u`を有効にしている場合は一時的に解除してください。Ollama APIを外部ネットワークに公開しないでください。

## 検証結果（2026年10月29日の一回の実行）

- LLMが指定順序 `CENTER → LEFT → RIGHT → CENTER` を正しく返した。
- MoveItが6段階すべての軌道を計画・実行し、関節の実測値を確認した。
- 第1関節の実測値：左 `−1.56499 rad`（約−89.67°）、右 `+1.56525 rad`（約+89.68°）。**左右の実測角度差は179.35°**。
- 6段階の目標角度と実測角度との差の最大絶対値は **0.00708 rad**（約0.41°）。
- 単体テスト22件がPASSした。この誤差は単一のシミュレーション実行における観測値であり、統計的な精度や実環境安全性の評価ではない。

## ファイル構成

| ファイル | 役割 |
|---|---|
| `sweep_demo.py` | **最終版**：LLMの出力検証、MoveIt起動、計画・観測ログ記録 |
| `moveit_ws/src/paia26_sweep/` | **最終版**：6段階の軌道計画・実行・関節到達検証 |
| `README_SWEEP.md` | 最終版のL字姿勢と制約の補足 |
| `tests/test_sweep_demo.py` | 最終版の巡回順序・ログ解析等のテスト |
| `final_demo.py` / `moveit_demo.py` | 以前の単関節／±0.4rad実演を再現する開発履歴 |
| `moveit_ws/src/paia26_moveit/` / `README_MOVEIT.md` | 以前のMoveIt巡回版（最終デモでは使用しない） |
| `ATTRIBUTION.md` | 第三者パッケージ・参考資料とライセンス |

`runs/`、シミュレータの動画やスクリーンショット、内部実行ログは公開リポジトリには含めません。

## 制約・出典・ライセンス

巡回は固定された方向・関節姿勢を実行するデモであり、実際の物体や異常を観測・認識・点検していません。LLMは許可済みの巡回順序の**解釈と一致確認**だけに使います。任意の命令を自動実行するエージェントではありません。対象ロボットと動作例、SRDF姿勢の出典は[ATTRIBUTION.md](ATTRIBUTION.md)に記載しています。

独自実装コードおよび文書は[Apache License 2.0](LICENSE)で公開しています。第三者パッケージとモデルには各配布元の条件が適用されます。
