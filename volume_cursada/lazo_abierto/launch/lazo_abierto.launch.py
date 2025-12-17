from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Use simulation time for all nodes
        Node(
            package='modelo_diferencial',
            executable='pioneer_odometry_node',
            name='pioneer_odometry',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='lazo_abierto',
            executable='trajectory_follower',
            name='trajectory_follower',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='lazo_abierto',
            executable='trajectory_generator',
            name='trajectory_generator',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'stepping': 0.1},
                #por default nos vino sin
                {'trajectory_type': 'spline'},
                #cambiar a 20 para el ejercicio 2
                {'total_time': 50.0},
                {'amplitude': 1.0},
                #cambiar a 5 para el ejercicio 2
                {'cycles': 1.0},
                # {'spline_waypoints': [
                # 0,0,0,0,
                # 0.0, 10.0, 0.0,  0.0,   
                # 10.0, 15.0, 5.0,  0.0,    
                # 15.0, 20.0, 10.0,  0.0     
                # ]}
                #los waypoints default:
                {'spline_waypoints': [
                    0., 0., 0., 0.,
                    10., 5., 0., 1.57,
                    20., 5., 5., 3.14,
                    30., 0., 5., 4.71,
                    40., 0., 0., 0.
                ]}
            ]
        )
    ])
# Note: each waypoint must have 4 values: time(sec), position_x(m), position_y(m), orientation(rad)