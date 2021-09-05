# arm_and_gripper
### まずちゃんとコンパイルする
```
rosdep update
rosdep install --from-paths src --ignore-src -r -y

```

### 起動
```
roslaunch arm_and_gripper spawn_gazebo_gripper_and_arm.launch
```
### グリッパを動かす

close
```
roslaunch arm_and_gripper gripper_move.launch value:=0.0195

```

open
```
roslaunch arm_and_gripper gripper_move.launch value:=0.0
```


