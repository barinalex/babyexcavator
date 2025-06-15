from launch import LaunchDescription
from launch_ros.actions import Node

# | Letter | Value |
# | ------ | ----- |
# | A      | 1     |
# | B      | 2     |
# | C      | 3     |
# | D      | 4     |
# | E      | 5     |
# | F      | 6     |
# | G      | 7     |
# | H      | 8     |
# | I      | 9     |
# | J      | 10    |
# | K      | 11    |
# | L      | 12    |
# | M      | 13    |
# | N      | 14    |
# | O      | 15    |
# | P      | 16    |
# | Q      | 17    |
# | R      | 18    |
# | S      | 19    |
# | T      | 20    |
# | U      | 21    |
# | V      | 22    |
# | W      | 23    |
# | X      | 24    |
# | Y      | 25    |
# | Z      | 26    |

def generate_launch_description():
    left_motor_node = Node(
        package='keyboard_drive',
        executable='keyboard_drive_node',
        name='keyboard_drive_left_motor',
        parameters=[{
            'forward': 23,      # W
            'release': 19,      # S
            'backward': 24,     # X
            'motor_name': 'left_track',
        }]
    )

    right_motor_node = Node(
        package='keyboard_drive',
        executable='keyboard_drive_node',
        name='keyboard_drive_right_motor',
        parameters=[{
            'forward': 5,      # E 
            'release': 4,      # D
            'backward': 3,     # C
            'motor_name': 'right_track',
        }]
    )

    platform = Node(
        package='keyboard_drive',
        executable='keyboard_drive_node',
        name='keyboard_drive_platform',
        parameters=[{
            'forward': 18,     # R
            'release': 6,      # F
            'backward': 22,    # V
            'motor_name': 'platform',
        }]
    )

    shoulder = Node(
        package='keyboard_drive',
        executable='keyboard_drive_node',
        name='keyboard_drive_shoulder',
        parameters=[{
            'forward': 20,     # T
            'release': 7,      # G
            'backward': 2,     # B
            'motor_name': 'shoulder',
        }]
    )

    arm = Node(
        package='keyboard_drive',
        executable='keyboard_drive_node',
        name='keyboard_drive_arm',
        parameters=[{
            'forward': 25,     # Y
            'release': 8,      # H
            'backward': 14,    # N
            'motor_name': 'arm',
        }]
    )

    bucket = Node(
        package='keyboard_drive',
        executable='keyboard_drive_node',
        name='keyboard_drive_bucket',
        parameters=[{
            'forward': 21,     # U
            'release': 10,     # J
            'backward': 13,    # M
            'motor_name': 'bucket',
        }]
    )

    return LaunchDescription([
        left_motor_node,
        right_motor_node,
        platform,
        shoulder,
        arm,
        bucket
    ])

