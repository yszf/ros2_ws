from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # 声明启动参数
    camera_config_path = LaunchConfiguration('camera_config_path')
    rtsp_config_path = LaunchConfiguration('rtsp_config_path')
    
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'camera_config_path',
            # 使用找到的Gemini 330系列配置文件
            default_value='$(find orbbec_camera)/config/gemini330Lg_series.yaml',
            description='Path to the camera configuration file'
        ),
        
        DeclareLaunchArgument(
            'rtsp_config_path',
            default_value='$(find orbbec_rtsp_bridge)/config/params.yaml',
            description='Path to the RTSP bridge configuration file'
        ),
        
        # 启动Orbbec相机驱动（使用实际的可执行文件名）
        Node(
            package='orbbec_camera',
            # 使用找到的可执行文件
            executable='orbbec_camera_node',
            name='orbbec_camera',
            output='screen',
            parameters=[camera_config_path],
        ),
        
        # 启动RTSP服务器
        Node(
            package='orbbec_rtsp_bridge',
            executable='orbbec_rtsp_server',
            name='orbbec_rtsp_server',
            output='screen',
            parameters=[rtsp_config_path],
        )
    ])