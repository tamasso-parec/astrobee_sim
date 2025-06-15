gz service -s /world/default/set_pose \
               --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
               --timeout 300 --req 'name: "astrobee", position: {z: 5.0}'
