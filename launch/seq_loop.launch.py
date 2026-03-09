from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():

    return LaunchDescription([

        # =============================
        # 1) Seq05
        # =============================
        Node(
            package='seq_loop',
            executable='seq05',
            name='seq05',
            output='screen'
        ),
        
        # =============================
        # 2) Loop05
        # =============================
        Node(
            package='seq_loop',
            executable='loop05',
            name='loop05',
            output='screen'
        ),
        
        # =============================
        # 3) Results plot (after 6 seconds, previous test is 5 seconds)
        # =============================
        TimerAction(
            period=6.0,  # delay in seconds
            actions=[
                Node(
                    package='seq_loop',
                    executable='plot_results.py',  # remove .py if installed as console_scripts
                    name='plot_results',
                    output='screen'
                )
            ]
        ),
    ])
