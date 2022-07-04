from fetch_api.base import Base
from fetch_api.torso import Torso
from fetch_api.arm_joints import ArmJoints
from fetch_api.arm import Arm
from fetch_api.gripper import Gripper

import rospy
import math
import requests
import time
import signal

# PoseNet names for each keypoint in the body
NOSE = 'nose'
LEFT_EYE = 'leftEye'
RIGHT_EYE = 'rightEye'
LEFT_EAR = 'leftEar'
RIGHT_EAR = 'rightEar'
LEFT_SHOULDER = 'leftShoulder'
RIGHT_SHOULDER = 'rightShoulder'
LEFT_ELBOW = 'leftElbow'
RIGHT_ELBOW = 'rightElbow'
LEFT_WRIST = 'leftWrist'
RIGHT_WRIST = 'rightWrist'
LEFT_HIP = 'leftHip'
RIGHT_HIP = 'rightHip'
LEFT_KNEE = 'leftKnee'
RIGHT_KNEE = 'rightKnee'
LEFT_ANKLE = 'leftAnkle'
RIGHT_ANKLE = 'rightAnkle'

# Actions to take
BASE_FORWARDS = 'BASE_FORWARDS'
BASE_BACKWARDS = 'BASE_BACKWARDS'
BASE_LEFT = 'BASE_LEFT'
BASE_RIGHT = 'BASE_RIGHT'
BASE_DOWN = 'BASE_DOWN'
BASE_UP = 'BASE_UP'
ARM_FORWARDS = 'ARM_FORWARDS'
ARM_BACKWARDS = 'ARM_BACKWARDS'
ARM_LEFT = 'ARM_LEFT'
ARM_RIGHT = 'ARM_RIGHT'
ARM_DOWN = 'ARM_DOWN'
ARM_UP = 'ARM_UP'
NO_ACTION = 'NO_ACTION'
GRIPPER_OPEN = 'GRIPPER_OPEN'
GRIPPER_CLOSE = 'GRIPPER_CLOSE'

# Position names
X = 'x'
Y = 'y'

# Set initial values for the arm and torso height
height = 0.0
elbow_flex = 0.0
wrist_roll = 0.0
wrist_flex = 0.0

# Initialize the node before the main file in order to set up the global api objects
rospy.init_node('robots_hw', anonymous=True)
print('Loading APIs...')

# Initialize fetch api objects
base = Base()
torso = Torso()
gripper = Gripper()
arm = Arm()
joints = ArmJoints.from_list([0, 0, 0, elbow_flex, 0, wrist_flex, wrist_roll])

# Set arm out straight ahead
arm.move_to_joints(joints)


def get_number_of_arms_down(kp):
    """Calculates the numbers of arms someone has down at their side

    Parameters
    ----------
    kp : dict
        The keypoints of the body, indexed by name, containing their x and y values

    Returns
    -------
    int
        The number of arms at someone's side, either 0, 1, or 2
    """
    return int(left_arm_is_down(kp)) + int(right_arm_is_down(kp))


def left_arm_is_down(kp):
    """Determines if someone's left arm is down at their side

    Parameters
    ----------
    kp : dict
        The keypoints of the body, indexed by name, containing their x and y values

    Returns
    -------
    bool
        True if someone's left arm is down at their side
    """
    # False if someone's wrist is more than their hip width away from their hip
    hip_width = dist(kp[LEFT_HIP], kp[RIGHT_HIP])
    left_wrist_to_hip_width = abs(kp[LEFT_WRIST][X] - kp[LEFT_HIP][X])
    if left_wrist_to_hip_width > hip_width:
        return False

    # False if someone's wrist is more than 1/2 of their forearm length above their hip
    left_forearm_length = dist(kp[LEFT_ELBOW], kp[LEFT_WRIST])
    left_wrist_to_hip_height = abs(kp[LEFT_WRIST][Y] - kp[LEFT_HIP][Y])
    if left_wrist_to_hip_height > (left_forearm_length / 2):
        return False

    return True


def right_arm_is_down(kp):
    """Determines if someone's right arm is down at their side

    Parameters
    ----------
    kp : dict
        The keypoints of the body, indexed by name, containing their x and y values

    Returns
    -------
    bool
        True if someone's right arm is down at their side
    """
    # False if someone's wrist is more than their hip width away from their hip
    hip_width = dist(kp[LEFT_HIP], kp[RIGHT_HIP])
    right_wrist_to_hip_width = abs(kp[RIGHT_WRIST][X] - kp[RIGHT_HIP][X])
    if right_wrist_to_hip_width > hip_width:
        return False

    # False if someone's wrist is more than 1/2 of their forearm length above their hip
    right_forearm_length = dist(kp[RIGHT_ELBOW], kp[RIGHT_WRIST])
    right_wrist_to_hip_height = abs(kp[RIGHT_WRIST][Y] - kp[RIGHT_HIP][Y])
    if right_wrist_to_hip_height > (right_forearm_length / 2):
        return False

    return True


def is_hands_together(kp):
    """Determines if someone has put their hands together

    Parameters
    ----------
    kp : dict
        The keypoints of the body, indexed by name, containing their x and y values

    Returns
    -------
    bool
        True if someone's hands are next to each other
    """
    # False if someone's wrists are more than their avg forearm length apart
    wrist_distance = dist(kp[LEFT_WRIST], kp[RIGHT_WRIST])
    avg_forearm_length = (dist(kp[LEFT_ELBOW], kp[LEFT_WRIST]) + dist(kp[RIGHT_ELBOW], kp[RIGHT_WRIST])) / 2
    if wrist_distance > avg_forearm_length:
        return False

    # False if someone's wrists are above their shoulders or below their hips, or if their wrists are not between
    # their shoulders
    if not (kp[LEFT_SHOULDER][Y] < kp[LEFT_WRIST][Y] < kp[LEFT_HIP][Y]) \
            or not (kp[RIGHT_SHOULDER][X] < kp[LEFT_WRIST][X] < kp[LEFT_SHOULDER][X]):
        return False
    if not (kp[RIGHT_SHOULDER][Y] < kp[RIGHT_WRIST][Y] < kp[RIGHT_HIP][Y]) \
            or not (kp[RIGHT_SHOULDER][X] < kp[RIGHT_WRIST][X] < kp[LEFT_SHOULDER][X]):
        return False

    return True


def is_base_down(kp):
    """Determines if someone's arms are pointing straight down, but away from the body

    Parameters
    ----------
    kp : dict
        The keypoints of the body, indexed by name, containing their x and y values

    Returns
    -------
    bool
        True if someone's left and right arms are both pointing straight down and away from the body
    """
    return is_base_down__left_arm(kp) and is_base_down__right_arm(kp)


def is_base_down__left_arm(kp):
    """Determine's if someone's left arm is pointing straight down and away from their body

    Parameters
    ----------
    kp : dict
        The keypoints of the body, indexed by name, containing their x and y values

    Returns
    -------
    bool
        True if someone's left arm is pointing straight down and away from their body
    """
    # False if someone's wrist isn't more than their hip width away from their hip
    hip_width = dist(kp[LEFT_HIP], kp[RIGHT_HIP])
    left_wrist_to_hip_width = abs(kp[LEFT_WRIST][X] - kp[LEFT_HIP][X])
    if left_wrist_to_hip_width < hip_width:
        return False

    # False if someone's wrist or elbow isn't angled more than 1/4 of their forearm length away from their shoulder
    left_forearm_length = dist(kp[LEFT_ELBOW], kp[LEFT_WRIST])
    left_wrist_shoulder_height = abs(kp[LEFT_WRIST][Y] - kp[LEFT_SHOULDER][Y])
    left_elbow_shoulder_height = abs(kp[LEFT_ELBOW][Y] - kp[LEFT_SHOULDER][Y])
    if left_wrist_shoulder_height < (left_forearm_length / 4) or left_elbow_shoulder_height < (left_forearm_length / 4):
        return False

    # False if someone's wrist or elbow is above their shoulder
    if kp[LEFT_WRIST][Y] < kp[LEFT_SHOULDER][Y] or kp[LEFT_ELBOW][Y] < kp[LEFT_SHOULDER][Y]:
        return False

    return True


def is_base_down__right_arm(kp):
    """Determine's if someone's right arm is pointing straight down and away from their body

    Parameters
    ----------
    kp : dict
        The keypoints of the body, indexed by name, containing their x and y values

    Returns
    -------
    bool
        True if someone's right arm is pointing straight down and away from their body
    """
    # False if someone's wrist isn't more than their hip width away from their hip
    hip_width = dist(kp[LEFT_HIP], kp[RIGHT_HIP])
    right_wrist_to_hip_width = abs(kp[RIGHT_WRIST][X] - kp[RIGHT_HIP][X])
    if right_wrist_to_hip_width < hip_width:
        return False

    # False if someone's wrist or elbow isn't angled more than 1/4 of their forearm length away from their shoulder
    right_forearm_length = dist(kp[RIGHT_ELBOW], kp[RIGHT_WRIST])
    right_wrist_shoulder_height = abs(kp[RIGHT_WRIST][Y] - kp[RIGHT_SHOULDER][Y])
    right_elbow_shoulder_height = abs(kp[RIGHT_ELBOW][Y] - kp[RIGHT_SHOULDER][Y])
    if right_wrist_shoulder_height < (right_forearm_length / 4) \
            or right_elbow_shoulder_height < (right_forearm_length / 4):
        return False

    # False if someone's wrist or elbow is above their shoulder
    if kp[RIGHT_WRIST][Y] < kp[RIGHT_SHOULDER][Y] or kp[RIGHT_ELBOW][Y] < kp[RIGHT_SHOULDER][Y]:
        return False

    return True


def is_base_up(kp):
    """Determine's if someone's arms are pointing straight up and away from their body

    Parameters
    ----------
    kp : dict
        The keypoints of the body, indexed by name, containing their x and y values

    Returns
    -------
    bool
        True if someone's arms are pointing straight up and away from their body
    """
    return is_base_up__left_arm(kp) and is_base_up__right_arm(kp)


def is_base_up__left_arm(kp):
    """Determine's if someone's left arm is pointing straight up and away from their body

    Parameters
    ----------
    kp : dict
        The keypoints of the body, indexed by name, containing their x and y values

    Returns
    -------
    bool
        True if someone's left arm is pointing straight up and away from their body
    """
    # False if someone's wrist or elbow isn't angled more than 1/4 of their forearm length away from their shoulder
    left_forearm_length = dist(kp[LEFT_ELBOW], kp[LEFT_WRIST])
    left_wrist_shoulder_height = abs(kp[LEFT_WRIST][Y] - kp[LEFT_SHOULDER][Y])
    left_elbow_shoulder_height = abs(kp[LEFT_ELBOW][Y] - kp[LEFT_SHOULDER][Y])
    if left_wrist_shoulder_height < (left_forearm_length / 4) or left_elbow_shoulder_height < (left_forearm_length / 4):
        return False

    # False if someone's wrist or elbow is below their shoulder
    if kp[LEFT_WRIST][Y] > kp[LEFT_SHOULDER][Y] or kp[LEFT_ELBOW][Y] > kp[LEFT_SHOULDER][Y]:
        return False

    return True


def is_base_up__right_arm(kp):
    """Determine's if someone's right arm is pointing straight up and away from their body

    Parameters
    ----------
    kp : dict
        The keypoints of the body, indexed by name, containing their x and y values

    Returns
    -------
    bool
        True if someone's right arm is pointing straight up and away from their body
    """
    # False if someone's wrist or elbow isn't angled more than 1/4 of their forearm length away from their shoulder
    right_forearm_length = dist(kp[RIGHT_ELBOW], kp[RIGHT_WRIST])
    right_wrist_shoulder_height = abs(kp[RIGHT_WRIST][Y] - kp[RIGHT_SHOULDER][Y])
    right_elbow_shoulder_height = abs(kp[RIGHT_ELBOW][Y] - kp[RIGHT_SHOULDER][Y])
    if right_wrist_shoulder_height < (right_forearm_length / 4) \
            or right_elbow_shoulder_height < (right_forearm_length / 4):
        return False

    # False if someone's wrist or elbow is below their shoulder
    if kp[RIGHT_WRIST][Y] > kp[RIGHT_SHOULDER][Y] or kp[RIGHT_ELBOW][Y] > kp[RIGHT_SHOULDER][Y]:
        return False

    return True


def is_base_forwards(kp):
    """Determine's if someone's arms are pointing straight out to the side

    Parameters
    ----------
    kp : dict
        The keypoints of the body, indexed by name, containing their x and y values

    Returns
    -------
    bool
        True if someone's arms are pointing straight out to the side
    """
    return is_base_forwards__left_arm(kp) and is_base_forwards__right_arm(kp)


def is_base_forwards__left_arm(kp):
    """Determine's if someone's left arm is pointing straight out to the side

    Parameters
    ----------
    kp : dict
        The keypoints of the body, indexed by name, containing their x and y values

    Returns
    -------
    bool
        True if someone's left arm is pointing straight out to the side
    """
    # False if someone's elbow or wrist is more than 1/4 of their forearm length away in height from their shoulder or
    # elbow
    left_forearm_length = dist(kp[LEFT_ELBOW], kp[LEFT_WRIST])
    left_wrist_elbow_height = abs(kp[LEFT_WRIST][Y] - kp[LEFT_ELBOW][Y])
    left_wrist_shoulder_height = abs(kp[LEFT_WRIST][Y] - kp[LEFT_SHOULDER][Y])
    left_elbow_shoulder_height = abs(kp[LEFT_ELBOW][Y] - kp[LEFT_SHOULDER][Y])
    if left_wrist_elbow_height > (left_forearm_length / 4) \
            or left_wrist_shoulder_height > (left_forearm_length / 4) \
            or left_elbow_shoulder_height > (left_forearm_length / 4):
        return False

    return True


def is_base_forwards__right_arm(kp):
    """Determine's if someone's right arm is pointing straight out to the side

    Parameters
    ----------
    kp : dict
        The keypoints of the body, indexed by name, containing their x and y values

    Returns
    -------
    bool
        True if someone's right arm is pointing straight out to the side
    """
    # False if someone's elbow or wrist is more than 1/4 of their forearm length away in height from their shoulder or
    # elbow
    right_forearm_length = dist(kp[RIGHT_ELBOW], kp[RIGHT_WRIST])
    right_wrist_elbow_height = abs(kp[RIGHT_WRIST][Y] - kp[RIGHT_ELBOW][Y])
    right_wrist_shoulder_height = abs(kp[RIGHT_WRIST][Y] - kp[RIGHT_SHOULDER][Y])
    right_elbow_shoulder_height = abs(kp[RIGHT_ELBOW][Y] - kp[RIGHT_SHOULDER][Y])
    if right_wrist_elbow_height > (right_forearm_length / 4) \
            or right_wrist_shoulder_height > (right_forearm_length / 4) \
            or right_elbow_shoulder_height > (right_forearm_length / 4):
        return False

    return True


def is_base_backwards(kp):
    """Determine's if someone's upper arms are pointing straight out, and their forearms are straight up

    Parameters
    ----------
    kp : dict
        The keypoints of the body, indexed by name, containing their x and y values

    Returns
    -------
    bool
        True if someone's upper arms are pointing straight out, and their forearms are straight up
    """
    return is_base_backwards__left_arm(kp) and is_base_backwards__right_arm(kp)


def is_base_backwards__left_arm(kp):
    """Determine's if someone's left upper arm is pointing straight out, and their forearm is straight up

    Parameters
    ----------
    kp : dict
        The keypoints of the body, indexed by name, containing their x and y values

    Returns
    -------
    bool
        True if someone's left upper arm is pointing straight out, and their forearm is straight up
    """
    # False if someone's elbow is more than 1/4 of their forearm away from their shoulder in height
    left_forearm_length = dist(kp[LEFT_ELBOW], kp[LEFT_WRIST])
    left_elbow_shoulder_height = abs(kp[LEFT_ELBOW][Y] - kp[LEFT_SHOULDER][Y])
    if left_elbow_shoulder_height > (left_forearm_length / 4):
        return False

    # False if someone's wrist is more than 1/4 of their forearm away from their elbow in width
    left_wrist_elbow_width = abs(kp[LEFT_WRIST][X] - kp[LEFT_ELBOW][X])
    if left_wrist_elbow_width > (left_forearm_length / 4):
        return False

    return True


def is_base_backwards__right_arm(kp):
    """Determine's if someone's right upper arm is pointing straight out, and their forearm is straight up

    Parameters
    ----------
    kp : dict
        The keypoints of the body, indexed by name, containing their x and y values

    Returns
    -------
    bool
        True if someone's right upper arm is pointing straight out, and their forearm is straight up
    """
    # False if someone's elbow is more than 1/4 of their forearm away from their shoulder in height
    right_forearm_length = dist(kp[RIGHT_ELBOW], kp[RIGHT_WRIST])
    right_elbow_shoulder_height = abs(kp[RIGHT_ELBOW][Y] - kp[RIGHT_SHOULDER][Y])
    if right_elbow_shoulder_height > (right_forearm_length / 4):
        return False

    # False if someone's wrist is more than 1/4 of their forearm away from their elbow in width
    right_wrist_elbow_width = abs(kp[RIGHT_WRIST][X] - kp[RIGHT_ELBOW][X])
    if right_wrist_elbow_width > (right_forearm_length / 4):
        return False

    return True


def is_base_left(kp):
    """Determine's if someone's upper arms are pointed straight out, and their forearms are angled left

    Parameters
    ----------
    kp : dict
        The keypoints of the body, indexed by name, containing their x and y values

    Returns
    -------
    bool
        True if someone's upper arms are pointed straight out, and their forearms are angled left
    """
    return is_base_left__left_arm(kp) and is_base_left__right_arm(kp)


def is_base_left__left_arm(kp):
    """Determine's if someone's left upper arm is pointed straight out, and their forearm is angled left

    Parameters
    ----------
    kp : dict
        The keypoints of the body, indexed by name, containing their x and y values

    Returns
    -------
    bool
        True if someone's left upper arm is pointed straight out, and their forearm is angled left
    """
    # False if someone's elbow is more than 1/4 of their forearm away from their shoulder in height
    left_forearm_length = dist(kp[LEFT_ELBOW], kp[LEFT_WRIST])
    left_elbow_shoulder_height = abs(kp[LEFT_ELBOW][Y] - kp[LEFT_SHOULDER][Y])
    if left_elbow_shoulder_height > (left_forearm_length / 4):
        return False

    # False if someone's wrist is less than 1/4 of their forearm away from their elbow in width or their wrist is not
    # to the left of their elbow
    left_wrist_elbow_width = abs(kp[LEFT_WRIST][X] - kp[LEFT_ELBOW][X])
    if left_wrist_elbow_width < (left_forearm_length / 4) \
            or not (kp[LEFT_SHOULDER][X] < kp[LEFT_WRIST][X] < kp[LEFT_ELBOW][X]):
        return False

    return True


def is_base_left__right_arm(kp):
    """Determine's if someone's right upper arm is pointed straight out, and their forearm is angled left

    Parameters
    ----------
    kp : dict
        The keypoints of the body, indexed by name, containing their x and y values

    Returns
    -------
    bool
        True if someone's right upper arm is pointed straight out, and their forearm is angled left
    """
    # False if someone's elbow is more than 1/4 of their forearm away from their shoulder in height
    right_forearm_length = dist(kp[RIGHT_ELBOW], kp[RIGHT_WRIST])
    right_elbow_shoulder_height = abs(kp[RIGHT_ELBOW][Y] - kp[RIGHT_SHOULDER][Y])
    if right_elbow_shoulder_height > (right_forearm_length / 4):
        return False

    # False if someone's wrist is less than 1/4 of their forearm away from their elbow in width or their wrist is not
    # to the left of their elbow
    right_wrist_elbow_width = abs(kp[RIGHT_WRIST][X] - kp[RIGHT_ELBOW][X])
    if right_wrist_elbow_width < (right_forearm_length / 4) or not (kp[RIGHT_WRIST][X] < kp[RIGHT_ELBOW][X]):
        return False

    return True


def is_base_right(kp):
    """Determine's if someone's upper arms are pointed straight out, and their forearms are angled right

    Parameters
    ----------
    kp : dict
        The keypoints of the body, indexed by name, containing their x and y values

    Returns
    -------
    bool
        True if someone's upper arms are pointed straight out, and their forearms are angled right
    """
    return is_base_right__left_arm(kp) and is_base_right__right_arm(kp)


def is_base_right__left_arm(kp):
    """Determine's if someone's left upper arm is pointed straight out, and their forearm is angled right

    Parameters
    ----------
    kp : dict
        The keypoints of the body, indexed by name, containing their x and y values

    Returns
    -------
    bool
        True if someone's left upper arm is pointed straight out, and their forearm is angled right
    """
    # False if someone's elbow is more than 1/4 of their forearm away from their shoulder in height
    left_elbow_shoulder_height = abs(kp[LEFT_ELBOW][Y] - kp[LEFT_SHOULDER][Y])
    left_forearm_length = dist(kp[LEFT_ELBOW], kp[LEFT_WRIST])
    if left_elbow_shoulder_height > (left_forearm_length / 4):
        return False

    # False if someone's wrist is less than 1/4 of their forearm away from their elbow in width or their wrist is not
    # to the right of their elbow
    left_wrist_elbow_width = abs(kp[LEFT_WRIST][X] - kp[LEFT_ELBOW][X])
    if left_wrist_elbow_width < (left_forearm_length / 4) or not (kp[LEFT_ELBOW][X] < kp[LEFT_WRIST][X]):
        return False

    return True


def is_base_right__right_arm(kp):
    """Determine's if someone's right upper arm is pointed straight out, and their forearm is angled right

    Parameters
    ----------
    kp : dict
        The keypoints of the body, indexed by name, containing their x and y values

    Returns
    -------
    bool
        True if someone's right upper arm is pointed straight out, and their forearm is angled right
    """
    # False if someone's elbow is more than 1/4 of their forearm away from their shoulder in height
    right_forearm_length = dist(kp[RIGHT_ELBOW], kp[RIGHT_WRIST])
    right_elbow_shoulder_height = abs(kp[RIGHT_ELBOW][Y] - kp[RIGHT_SHOULDER][Y])
    if right_elbow_shoulder_height > (right_forearm_length / 4):
        return False

    # False if someone's wrist is less than 1/4 of their forearm away from their elbow in width or their wrist is not
    # to the right of their elbow
    right_wrist_elbow_width = abs(kp[RIGHT_WRIST][X] - kp[RIGHT_ELBOW][X])
    if right_wrist_elbow_width < (right_forearm_length / 4)  \
            or not (kp[RIGHT_ELBOW][X] < kp[RIGHT_WRIST][X] < kp[RIGHT_SHOULDER][X]):
        return False

    return True


def is_arm_forwards(kp):
    """Determines if someone has exactly one of their arms pointing straight out to the side

    Parameters
    ----------
    kp : dict
        The keypoints of the body, indexed by name, containing their x and y values

    Returns
    -------
    bool
        True if someone has exactly one of their arms pointing straight out to the side
    """
    return (is_base_forwards__left_arm(kp) and right_arm_is_down(kp)) or (is_base_forwards__right_arm(kp) and left_arm_is_down(kp))


def is_arm_backwards(kp):
    """Determines if someone has exactly one of their upper arms pointing straight out to the side, and their
    forearm straight up

    Parameters
    ----------
    kp : dict
        The keypoints of the body, indexed by name, containing their x and y values

    Returns
    -------
    bool
        True if someone has exactly one of their upper arms pointing straight out to the side, and their
        forearm straight up
    """
    return (is_base_backwards__left_arm(kp) and right_arm_is_down(kp)) or (is_base_backwards__right_arm(kp) and left_arm_is_down(kp))


def is_arm_left(kp):
    """Determines if someone has exactly one of their upper arms pointing straight out to the side, and their forearm
    angled left

    Parameters
    ----------
    kp : dict
        The keypoints of the body, indexed by name, containing their x and y values

    Returns
    -------
    bool
        True if someone has exactly one of their upper arms pointing straight out to the side, and their forearm
        angled left
    """
    return (is_base_left__left_arm(kp) and right_arm_is_down(kp)) or (is_base_left__right_arm(kp) and left_arm_is_down(kp))


def is_arm_right(kp):
    """Determines if someone has exactly one of their upper arms pointing straight out to the side, and their forearm
    angled right

    Parameters
    ----------
    kp : dict
        The keypoints of the body, indexed by name, containing their x and y values

    Returns
    -------
    bool
        True if someone has exactly one of their upper arms pointing straight out to the side, and their forearm
        angled right
    """
    return (is_base_right__left_arm(kp) and right_arm_is_down(kp)) or (is_base_right__right_arm(kp) and left_arm_is_down(kp))


def is_arm_down(kp):
    """Determines if someone has exactly one of their arms pointing straight down and away from their body

    Parameters
    ----------
    kp : dict
        The keypoints of the body, indexed by name, containing their x and y values

    Returns
    -------
    bool
        True if someone has exactly one of their arms pointing straight down and away from their body
    """
    return (is_base_down__left_arm(kp) and right_arm_is_down(kp)) or (is_base_down__right_arm(kp) and left_arm_is_down(kp))


def is_arm_up(kp):
    """Determines if someone has exactly one of their arms pointing straight up and away from their body

    Parameters
    ----------
    kp : dict
        The keypoints of the body, indexed by name, containing their x and y values

    Returns
    -------
    bool
        True if someone has exactly one of their arms pointing straight up and away from their body
    """
    return (is_base_up__left_arm(kp) and right_arm_is_down(kp)) or (is_base_up__right_arm(kp) and left_arm_is_down(kp))


def get_base_action_to_take(kp):
    """Calculates the correct base action to take based on the given keypoints from a pose. Base actions imply that
    someone has both of their arms away from their side.

    Parameters
    ----------
    kp : dict
        The keypoints of the body, indexed by name, containing their x and y values

    Returns
    -------
    str
        The base action for the robot to take
    """
    if is_base_forwards(kp):
        return BASE_FORWARDS
    elif is_base_backwards(kp):
        return BASE_BACKWARDS
    elif is_base_left(kp):
        return BASE_LEFT
    elif is_base_right(kp):
        return BASE_RIGHT
    elif is_base_down(kp):
        return BASE_DOWN
    elif is_base_up(kp):
        return BASE_UP
    else:
        return NO_ACTION


def get_arm_action_to_take(kp):
    """Calculates the correct arm action to take based on the given keypoints from a pose. Arm actions imply that
    someone has exactly one of their arms away from their side.

    Parameters
    ----------
    kp : dict
        The keypoints of the body, indexed by name, containing their x and y values

    Returns
    -------
    str
        The arm action for the robot to take
    """
    if is_arm_forwards(kp):
        return ARM_FORWARDS
    elif is_arm_backwards(kp):
        return ARM_BACKWARDS
    elif is_arm_left(kp):
        return ARM_LEFT
    elif is_arm_right(kp):
        return ARM_RIGHT
    elif is_arm_down(kp):
        return ARM_DOWN
    elif is_arm_up(kp):
        return ARM_UP
    else:
        return NO_ACTION


def get_gripper_action_to_take(kp):
    """Calculates the correct gripper action to take, given that someone's hands are together

    Parameters
    ----------
    kp : dict
        The keypoints of the body, indexed by name, containing their x and y values

    Returns
    -------
    str
        The correct gripper action to take, either opening or closing
    """
    # open the gripper if the hands are together above the elbows, and close otherwise
    if kp[LEFT_WRIST][Y] < kp[LEFT_ELBOW][Y] and kp[RIGHT_WRIST][Y] < kp[RIGHT_ELBOW][Y]:
        return GRIPPER_OPEN
    else:
        return GRIPPER_CLOSE


def get_action_to_take(kp):
    """Calculates whether the action to take belongs to the base, arm, or gripper

    Parameters
    ----------
    kp : dict
        The keypoints of the body, indexed by name, containing their x and y values

    Returns
    -------
    str
        The final action to take
    """
    number_of_arms_down = get_number_of_arms_down(kp)

    is_base_action = number_of_arms_down == 0
    is_arm_action = number_of_arms_down == 1
    is_no_action = number_of_arms_down == 2
    is_gripper_action = is_hands_together(kp)

    if is_no_action:
        return NO_ACTION
    elif is_gripper_action:
        return get_gripper_action_to_take(kp)
    elif is_base_action:
        return get_base_action_to_take(kp)
    elif is_arm_action:
        return get_arm_action_to_take(kp)


def execute_action(action):
    """Calls the fetch api to perform a given action

    Parameters
    ----------
    action : str
        The action for the fetch robot to perform
    """
    # Original values set at the beginning of the program
    global height
    global elbow_flex
    global wrist_flex
    global wrist_roll

    # Values for setting speed and distance for actions
    base_trans_dist = 0.1
    base_trans_speed = 0.8
    base_rot_dist = 10
    base_rot_speed = 0.5
    base_height_dist = 0.1
    elbow_flex_dist = 10
    wrist_flex_dist = 10
    wrist_roll_dist = 10

    # Do nothing
    if action == NO_ACTION:
        return

    # Gripper actions
    elif action == GRIPPER_OPEN:
        gripper.open()
        return
    elif action == GRIPPER_CLOSE:
        gripper.close()
        return

    # Base actions
    elif action == BASE_FORWARDS:
        base.go_forward(base_trans_dist, base_trans_speed)
        return
    elif action == BASE_BACKWARDS:
        base.go_forward(-1 * base_trans_dist, base_trans_speed)
        return
    elif action == BASE_DOWN:
        height = max(height - base_height_dist, 0.0)
        torso.set_height(height)
        return
    elif action == BASE_UP:
        height = min(height + base_height_dist, 4.0)
        torso.set_height(height)
        return
    elif action == BASE_LEFT:
        base.turn(math.radians(base_rot_dist), base_rot_speed)
        return
    elif action == BASE_RIGHT:
        base.turn(math.radians(-1 * base_rot_dist), base_rot_speed)
        return

    # Arm actions
    elif action == ARM_FORWARDS:
        elbow_flex = elbow_flex + elbow_flex_dist
    elif action == ARM_BACKWARDS:
        elbow_flex = elbow_flex - elbow_flex_dist
    elif action == ARM_DOWN:
        wrist_flex = wrist_flex + wrist_flex_dist
    elif action == ARM_UP:
        wrist_flex = wrist_flex - wrist_flex_dist
    elif action == ARM_LEFT:
        wrist_roll = wrist_roll - wrist_roll_dist
    elif action == ARM_RIGHT:
        wrist_roll = wrist_roll + wrist_roll_dist
    
    # Move the arm according to its new values
    joints = ArmJoints.from_list(
        [0, 0, 0, math.radians(elbow_flex), 0, math.radians(wrist_flex), math.radians(wrist_roll)])
    arm.move_to_joints(joints)


def process_keypoints(keypoints):
    """Alters the keypoints dictionary to be able to access keypoints positions by their name

    Parameters
    ----------
    keypoints : dict
        The original dictionary of keypoints, which is indexed by numbers, not names

    Returns
    -------
    dict
        The new keypoints dictionary, indexed by names
    """
    kp = {}

    for (key, value) in keypoints.items():
        kp[value['part']] = value['position']

    return kp


def dist(point_1, point_2):
    """Calculates the Euclidean distance between two points

    Parameters
    ----------
    point_1 : dict
        The x and y values of point 1, accessible via 'x' and 'y' keys
    point_2 : dict
        The x and y values of point 2, accessible via 'x' and 'y' keys

    Returns
    -------
    float
        The Euclidean distance between the two points
    """
    return math.sqrt(((point_1[X] - point_2[X]) ** 2) + ((point_1[Y] - point_2[Y]) ** 2))


def shutdown():
    """Handles SIGTERM commands to the program and shuts down the python execution

    Raises
    ------
    KeyboardInterrupt
        Raise a KeyboardInterrupt in order to print out the stack trace for the program on Ctrl+c
    """
    raise KeyboardInterrupt


if __name__ == "__main__":
    """Fetches the keypoints data from the web app once a second, finds the most frequent pose action, and executes
    it on the robot
    """
    # Gandle Ctrl+c from user to shut down the program and throw a KeyboardInterrupt exception
    signal.signal(signal.SIGTERM, shutdown)
    WEB_APP_URL = 'https://tranquil-thicket-20301.herokuapp.com/data'
    print('Ready to execute actions')
    print('Start posing in')

    for second in range(2, -1, -1):
        print(str(second + 1))
        time.sleep(1)

    # Run forever, until the user ends the program
    while (True):
        # Every action calculated in a second of video (one every 0.1 seconds, so 10 actions)
        calculated_actions = []
        # Keypoints from the previous decisecond
        previous_keypoints = {}

        for decisecond in range(10):

            # GET the keypoints from the web app, which are updated continuously if it is running
            try:
                request = requests.get(WEB_APP_URL)
                keypoints = request.json()

            # If there's a problem, don't take an action
            except Exception as e:
                calculated_actions.append(NO_ACTION)
                print(e)
                continue

            # If the keypoints haven't changed, then the web app currently isn't detecting a pose, so don't take an action
            if keypoints == previous_keypoints:
                calculated_actions.append(NO_ACTION)
                continue

            # Otherwise, calculate which action to take and add it to calculated_actions
            if keypoints:
                action = get_action_to_take(process_keypoints(keypoints))
                calculated_actions.append(action)

            # If the keypoints came back empty, don't take an action
            else:
                calculated_actions.append(NO_ACTION)
                continue

            # Wait a little bit before making another API call
            time.sleep(.1)

        # Find the most common action calculated in the second and execute it
        mode_action = max(set(calculated_actions), key=calculated_actions.count)
        print('==============================')
        print('Next action: ' + mode_action)
        print('Executing action...')
        execute_action(mode_action)
        print('Done')
        print('==============================')
