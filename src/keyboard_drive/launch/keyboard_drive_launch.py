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
    # Left motor node with keys: Q (forward), A (release), Z (backward)
    left_motor_node = Node(
        package='keyboard_drive',
        executable='keyboard_drive_node',
        name='keyboard_drive_left_motor',
        parameters=[{
            'forward': 17,      # Q
            'release': 1,      # A
            'backward': 26,     # Z
            'topic_name': '/left_track/cmd_vel',
        }]
    )

    # Right motor node with keys: W (forward), S (release), X (backward)
    right_motor_node = Node(
        package='keyboard_drive',
        executable='keyboard_drive_node',
        name='keyboard_drive_right_motor',
        parameters=[{
            'forward': 23,      # W
            'release': 19,      # S
            'backward': 24,     # X
            'topic_name': '/right_track/cmd_vel',
        }]
    )

    return LaunchDescription([
        left_motor_node,
        right_motor_node,
    ])

