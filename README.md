# b3w_walker

B3冬に3dプリンタでヒューマノイドロボットを作成した。moveitで静的なバランスのみを考慮した歩行動作計画を行い一歩踏み出し動作を行ったが、途中で転倒した。

ros-kinetic-moveit-visual-tools
ros-kinetic-controller-manager
ros-kinetic-transmission-interface
use Naoki-Hiraoka/hrl_kinematics/tree/develop

ros-kinetic-position-controllers
ros-kinetic-joint-trajectory-controller
ros-kinetic-gazebo-ros-control

env GAZEBO_MODEL_PATH にこのディレクトリのパスを追加

ハードウェアに搭載したraspberry pi3で
```roslaunch b3w_walker b3w_walker_bringup.launch```
またはノートPC上で
```roslaunch b3w_walker b3w_walker_gazebo.launch```

そして、
```roslaunch b3w_walker b3w_walker_planner.launch```

その後、
```rosrun b3w_humanoid tutorial2```