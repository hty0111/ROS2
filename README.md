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
ros2 interface show <msg>/<srv>/<action>	# 查看定义
ros2 interface proto <msg>/<srv>/<action>	# 查看原型，可用于.idl文件
ros2 interface package <pkg>	# 查看功能包中用到的所有接口
```

### Param

#### 外部声明 .launch.py

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='param',
            default_value='0.1',
            description=''
        ),
        Node(
            package='pkg',
            executable='exe',
            name='node',
            output='screen',
            parameters=[
                {'param': LaunchConfiguration('param')}
            ]
        )
    ])
```

#### 内部调用 .py

```shell
# Node中只能使用get_parameter()获取参数，且必须先调用declare_parameter()声明参数
class SelfNode(Node):
    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)
	self.declare_parameter('param', '0.1')
	param = self.get_parameter('param').get_parameter_value().string_value
	self.get_logger().info(param)

# CompatibleNode具有ROS1兼容性，可以用get_param()获取参数，且不需要提前声明
class SelfNode(CompatibleNode):
    def __init__(self, name, **kwargs):
	super().__init__(name, **kwargs)
	param = self.get_param('param', '0.1')
	# param = self.get_parameter('param').get_parameter_value().string_value
	self.get_logger().info(param)
```

==注意==：

* 删除功能包后AMENT_PREFIX_PATH和CMAKE_PREFIX_PATH由于环境变量未更新会产生warning，新开一个终端重新编译即可。
* 一个功能包只能用cmake/python，因此自定义消息最好额外放在其他功能包中，以便共用。
