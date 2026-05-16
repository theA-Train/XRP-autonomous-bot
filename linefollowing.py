from drivetrain import *
from commands2 import SelectCommand

drivetrain = Drivetrain()
drive_straight = Drive_To_Distance(drivetrain, 0.4).repeatedly()
turn = Rotate_Drivetrain(drivetrain, 90).repeatedly()
high_reflectance = 0.8


def sensor_check():
    if drivetrain.get_left_reflectance() > high_reflectance and drivetrain.get_left_reflectance() > high_reflectance:
        return "straight"
    else:
        return "turn"
    
line_following_selector = SelectCommand(
    {
        "straight" : drive_straight,
        "turn": turn
    },
    sensor_check
)
    