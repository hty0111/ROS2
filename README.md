# ROS2官网 & 古月居《ROS2入门21讲》

2022.7.19



### Package

```shell
# create
cd dev_ws/src
ros2 pkg create --build-type ament_cmake [--dependencies rclcpp std_msgs] \ 
	[--node-name <node>] <pkg> 	# C++
ros2 pkg create --build-type ament_python <pkg>	# python
# build
cd dev_ws
source install/local_setup.sh
colcon build [--packages-select <pkg>]
# cd
colcon_cd <pkg>
```

### Node

```shell
ros2 node list
ros2 node info <node>
```

### Topic

```shell
ros2 topic list [-t]	# 同时显示msg
ros2 topic echo <topic>	# 显示话题下的消息数据
ros2 topic info <topic> # 显示与话题相关的订阅方和发布方
ros2 topic pub [--once/-1]/[-r/--rate <N>] <topic> <msg> "<args>"
ros2 topic hz <topic>	# 显示发布频率
```

### Service

```shell
ros2 service list [-t]		# 同时显示srv
ros2 service type <service>	# 显示服务对应的消息类型
ros2 service find <srv>		# 显示调用消息的服务
ros2 service call <service> <srv> "<args>"
```

### Msg/Srv

```shell
ros2 interface show <msg>/<srv>/<action>
ros2 interface package <pkg>	# 查看功能包中用到的所有接口
```

==注意==：

* 删除功能包后AMENT_PREFIX_PATH和CMAKE_PREFIX_PATH由于环境变量未更新会产生warning，新开一个终端重新编译即可。
* 一个功能包只能用cmake/python，因此自定义消息最好额外放在其他功能包中，以便共用。
