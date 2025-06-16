gz service -s /world/default/set_pose \
               --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
               --timeout 300 --req 'name: "astrobee", position: {z: 5.0}'

ros2 topic pub /astrobee/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}'


ros2 topic pub /chaser/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0,y: 0.0, z: 0.0}, angular: {x: 0.0, y: 6.98, z: 0.01}}'
ros2 topic pub /target/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0,y: 0.0, z: 0.0}, angular: {x: 0.0, y: 6.98, z: 0.01}}'
