# ros_pc_filters
ROS×PCLのフィルターをかき集めました

1. pass through filter
  指定範囲に存在する点群のみを抽出するフィルター
2. euclidian fiter
  点群の集合（クラスター）のリストを作成しそのクラスターごとにそこに含まれるポイントクラウドのインデックスのリストを作ってくれるスーパー優秀のフィルター
  3次元物体検出などを行うときによく使われるやつ
 
3. voxel grid filter
  センサーから取得したポイントクラウドを1辺grid_sizeごとの格子点マップに置き換えてくれるフィルター
  そのままの点群だと処理に時間がかかってしまうがこいつを使えばむらなく均等にダウンサンプリングされる（マジ有能）

4. downsampling filter
  シンプルに取得する点群を減らすフィルター　この場合はセンサーから近い方が密になり遠い方が粗になる
  
5. median fiter
  メディアンフィルター
  
6. kd tree filter
  近傍点探索を行うノード
  
7. sac segmentation filter
  いろんな点集合を抽出or除外できるフィルタ
  - plane filter 平面の抽出or削除
  - sphere filter 球体の抽出or削除
  - cylinder filter 円柱の抽出or削除
9. その他追加予定
