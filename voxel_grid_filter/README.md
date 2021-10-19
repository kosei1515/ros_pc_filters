# voxel gird filter

センサーから取得したポイントクラウドを均等にダウンサンプリングするフィルター

## parameters
launch/voxel_grid_filter_node.launch内のパラメータを変更することでleaf_sizeを変更できる．

```
lead_size_x x軸方向のleaf size      default 0.1
lead_size_y y軸方向のleaf size      default 0.1
lead_size_z z軸方向のleaf size      default 0.1
```
## code
```
roslaunch voxel_grid_filter voxel_grid_filter_node.launch 
```
