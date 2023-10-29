import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import  LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):

    rae_prefix = get_package_share_directory('rae_camera')
    name = LaunchConfiguration('name').perform(context)
    laserscan_config = os.path.join(
        rae_prefix,
        'config',
        'laserscan_kinect.yaml'
    )

    parameters = [
        {
            'frame_id': 'base_footprint',
            'subscribe_rgbd': True,
            'approx_sync': True,
            'Mem/BinDataKept': 'true',
            'DbSqlite3/InMemory': 'false',
            'Grid/NormalsSegmentation': 'false',
            'Grid/MaxGroundHeight': '0.02',
            'Grid/MaxObstacleHeight': '0.1',
            'Grid/FromDepth': True,
            'Grid/RangeMax': '2.0',
            'Grid/3D': 'false',
            'Grid/RayTracing': 'true',
            'Reg/Force3DoF': 'true',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            'qos': 1
        }
    ]

    remappings = [
        ('odom', '/odometry/filtered'),
        ('rgbd_image', '/rae/front/rgbd_image'),
    ]

    return [


        LoadComposableNodes(
            target_container=name+'_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='rtabmap_sync',
                    plugin='rtabmap_sync::RGBDSync',
                    name='rgbd_sync',
                    parameters=[{
                        'approx_sync': True,
                        'approx_sync_max_interval': 0.01,
                        'qos': 1
                    }],
                    remappings=[
                        ('rgb/image', name+'/left/image_rect'),
                        ('rgb/camera_info', name+'/left/camera_info'),
                        ('depth/image', name+'/stereo_front/image_raw'),
                        ('rgbd_image', name+'/front/rgbd_image'),
                        ('rgbd_image/compressed', name+'/front/rgbd_image/compressed'),
                    ],
                ),
                ComposableNode(
                    package='rtabmap_slam',
                    plugin='rtabmap_slam::CoreWrapper',
                    name='rtabmap',
                    parameters=parameters,
                    remappings=remappings,
                )
            ]),

        LoadComposableNodes(
            target_container=name+'_container',
            composable_node_descriptions=[
                    ComposableNode(
                        package='laserscan_kinect',
                        plugin='laserscan_kinect::LaserScanKinectNode',
                        name='laserscan_kinect_front',
                        parameters=[laserscan_config],
                        remappings=[
                            ('/image', name+'/stereo_front/image_raw'),
                            ('/camera_info', name+'/stereo_front/camera_info'),
                            ('/scan', name+'/scan_front'),
                            ('/debug_image', name+'/debug_image_front'),
                            ('/debug_image/compressed', name+'/debug_image_front/compressed')
                        ]
                    ),
                    ComposableNode(
                        package='laserscan_kinect',
                        plugin='laserscan_kinect::LaserScanKinectNode',
                        name='laserscan_kinect_back',
                        parameters=[laserscan_config],
                        remappings=[
                            ('/image', name+'/stereo_back/image_raw'),
                            ('/camera_info', name+'/stereo_back/camera_info'),
                            ('/scan', name+'/scan_back'),
                            ('/debug_image', name+'/debug_image_back'),
                            ('/debug_image/compressed', name+'/debug_image_back/compressed')
                        ]
                    )
            ]
        )  
        ]


def generate_launch_description():
    rae_prefix = get_package_share_directory('rae_bringup')
    declared_arguments = [
        DeclareLaunchArgument('name', default_value='rae'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            rae_prefix, 'config', 'camera.yaml')),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
