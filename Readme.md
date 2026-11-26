# opera_behavior_tree

Author: itsuka-to

mst110cr_ros2 (branch: mst110cr_2-route-server) で必要なパッケージです。
## Route入口での前後進判定

`AlignPathOrientations` の `mode="route_entry"` は、`mst110cr_remote` が使う
`enable_route_compute_and_track_route_graph_2.xml` のRoute追従中に毎tick実行します。
計画時の車体姿勢ではなく、`GetCurrentPose` で取得した入口到達時の姿勢を使います。

- 平滑化後のRoute本体の先頭からXY距離0.5 m以内で判定します。
- Route本体の先頭から弧長1.0 m先までの弦方向を入口の接線近似とします。
  短いRouteでは終端までを使い、距離のないRouteでは判定しません。
- 車体yawとの角度差が90度を超えれば後進、それ以下は前進です。
  厳密に90度の場合は前進を選びます。
- 判定後はRoute本体の姿勢を接線方向（前進）または接線方向+π（後進）に設定します。
  位置、first/last-mile接続部分、指定ゴール姿勢と終端の重複点は維持します。
- `route_direction` をblackboardに保持し、同じゴールへの再計画やcontroller recoveryで
  カーブ途中の方向を決め直しません。新しいゴール、次のnavigation開始、同一エリア内の
  非Routeモードではリセットします。途中の任意の切り返し指定には対応しません。
- 入口前は既存の経路姿勢を使用します。経路から前後進を判断するための準備処理であり、
  速度指令の符号を強制する処理ではありません。

XMLの `entry_tolerance` と `tangent_distance` で距離を調整できます。
既定の `mode="forward"` は従来どおり平滑化前の正規化用として使い、入口での姿勢設定は平滑化後に
行います。PathAlign/PathAngleの既存設定は変更しません。

Routeの範囲は、接続前のRouteと平滑化前の結合経路の点列から特定します。
現在の `ConcatenatePaths` と `SimpleSmoother` が点数・点順序を維持する前提です。
点数が変わるスムーザーへ変更する場合は範囲対応の実装も変更してください。
一致する範囲が曖昧、座標系が不一致、入力が不正な場合はFAILUREを返します。
計画途中の入力が混ざらないよう、BTは平滑化と最終姿勢の設定が完了してから
`route_base_path` / `route_body_path` / `route_unsmoothed_path` / `route_plan_goal` を更新します。

ビルド後にnavigationを再起動すると更新したプラグインが読み込まれます。
テストはROSを初期化せず、BTプラグインを直接読み込んで実行できます。

```bash
colcon build --packages-select opera_behavior_tree mst110cr_navigation --symlink-install
colcon test --packages-select opera_behavior_tree
```
