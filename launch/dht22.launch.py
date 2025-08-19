from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    chip_path = LaunchConfiguration('chip_path')
    gpio_line = LaunchConfiguration('gpio_line')
    rate_hz   = LaunchConfiguration('rate_hz')
    tries     = LaunchConfiguration('tries_per_sample')
    frame_id  = LaunchConfiguration('frame_id')

    return LaunchDescription([
        DeclareLaunchArgument('chip_path', default_value='/dev/gpiochip4',
                              description='Ruta del gpiochip (RPi5: /dev/gpiochip4)'),
        DeclareLaunchArgument('gpio_line', default_value='12',
                              description='Línea GPIO (offset BCM) p.ej. 12 = GPIO12'),
        DeclareLaunchArgument('rate_hz', default_value='0.5',
                              description='Frecuencia (Hz). Mínimo efectivo: 0.5 Hz'),
        DeclareLaunchArgument('tries_per_sample', default_value='4',
                              description='Reintentos por muestra'),
        DeclareLaunchArgument('frame_id', default_value='dht22_link',
                              description='Frame ID'),

        Node(
            package='dht22_sensor',
            executable='dht22_node',
            name='dht22_node',
            output='screen',
            parameters=[{
                'chip_path': chip_path,
                'gpio_line': ParameterValue(gpio_line, value_type=int),
                'rate_hz': ParameterValue(rate_hz, value_type=float),
                'tries_per_sample': ParameterValue(tries, value_type=int),
                'frame_id': frame_id,
            }]
        )
    ])
