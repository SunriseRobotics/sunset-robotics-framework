from architecture.scheduler import Scheduler
from architecture.architecture_relationships import Topic, Message, Subscriber, Command, DelayCommand
from sunset_math.geometry import SE3, SO3
import math
import random
import numpy as np

is_rpi = True

if is_rpi:
    import serial
    import RPi.GPIO as GPIO

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(14, GPIO.IN)
    ser = serial.Serial('/dev/ttyACM0', 115200)
else:
    ser = None

L1 = 0.242
L2 = 0.220
L3 = 0.29531605

# the arm does not spin directly in it's center of rotation
X_offset = 0.04
Y_offset = 0

scanner_x_rel = 0.085
scanner_z_rel = -0.24
scanner_y_rel = 0

counts_per_revolution = 8192  # according to rev documentation

debug_encoders = False
debug_arm_angles = False
debug_serial_raw = False
debug_serial_message = False
debug_scanner_imu = False
debug_gpio_pin = False
# the final output position, make this true and the others false for OCT usage (only if you don't want to rely on GPIO)
show_final_pos = False
# true if you want to print the scanner position when the gpio output is high
print_on_gpio_output = True


# kinematics of the 2 joints
def solve_forward_kinematics(theta1, theta2):
    x = np.sin(theta1) * L2 + np.sin(theta2) * L3
    z = L1 - np.cos(theta1) * L2 - np.cos(theta2) * L3
    return SE3.from_euler_and_translation(0, theta2, 0, x, 0, z)


# kinematics of first joint, only for visualization purposes, not actually used for computation of pose
def solve_forward_kinematics_joint1(theta1):
    x = np.sin(theta1) * L2
    z = L1 - np.cos(theta1) * L2
    return SE3.from_euler_and_translation(0, theta1, 0, x, 0, z)


def encoder_counts_to_radians(counts):
    return (counts / counts_per_revolution) * 2 * np.pi


class SerialBusReader(Topic):
    def __init__(self, topic_name="Serial", is_sim=False):
        super().__init__(topic_name, is_sim)
        self.replace_message_with_log = True
        self.previous = [0 for i in range(9)]

        if is_sim:
            self.ser = None
            return

        self.ser = serial.Serial('/dev/ttyACM0', 115200)
        if debug_serial_raw:
            print(self.ser)

    def generate_messages_periodic(self):
        serial_text = ser.readline()
        serial_text = str(serial_text)[2:-3]
        data = serial_text.split(',')
        if debug_serial_raw:
            print(serial_text)
        if len(data) < 9:
            data = self.previous
        else:
            try:
                data = [float(datum) for datum in data]
                self.previous = data
            except:
                data = self.previous
        print(data)
        message_dict = {
            "encoder_1": data[0], "encoder_2": data[1], "imu_orientation_x_deg": data[2],
            "imu_orientation_y_deg": data[3], "imu_orientation_z_deg": data[4],
            "scanner_q_w": data[5], "scanner_q_y": data[6], "scanner_q_x": data[7],
            "scanner_q_z": data[8]
        }
        if debug_serial_message:
            print(message_dict)
        return message_dict


class IMU(Topic):
    def __init__(self, topic_name="IMU_ORIENTATION", is_sim=False):
        super().__init__(topic_name, is_sim)
        self.imu_message = {"X": 0, "Y": 0, "Z": 0}

    def subscriber_periodic(self):
        self.imu_message["Z"] = math.radians(self.messages["Serial"].message["imu_orientation_x_deg"])
        self.imu_message["Y"] = math.radians(self.messages["Serial"].message["imu_orientation_y_deg"])
        self.imu_message["X"] = math.radians(self.messages["Serial"].message["imu_orientation_z_deg"])

    def generate_messages_periodic(self):
        return self.imu_message


class Scanner_IMU(Topic):
    def __init__(self, topic_name="SCANNER_ORIENTATION", is_sim=False):
        super().__init__(topic_name, is_sim)
        self.imu_message = {"X": 0, "Y": 0, "Z": 0, "W": 0}
        # not used lol
        self.x_offset = 0
        self.y_offset = 0
        self.z_offset = 0

    def subscriber_periodic(self):
        self.imu_message["Z"] = self.messages["Serial"].message["scanner_q_z"]
        self.imu_message["Y"] = self.messages["Serial"].message["scanner_q_y"]
        self.imu_message["X"] = self.messages["Serial"].message["scanner_q_x"]
        self.imu_message["W"] = self.messages["Serial"].message["scanner_q_w"]

    def generate_messages_periodic(self):
        if debug_scanner_imu:
            print(self.imu_message)
        return self.imu_message


class Encoders(Topic):
    def __init__(self, topic_name="Encoders", is_sim=False):
        super().__init__(topic_name, is_sim)
        self.encoder_message = {"Encoder1": 0, "Encoder2": 0}
        self.zero_offset_1 = 0
        self.zero_offset_2 = 0

    def subscriber_periodic(self):
        self.encoder_message["Encoder1"] = (self.messages["Serial"].message["encoder_1"] - self.zero_offset_1)
        self.encoder_message["Encoder2"] = self.messages["Serial"].message["encoder_2"] - self.zero_offset_2
        # swap
        temp = self.encoder_message["Encoder1"]
        self.encoder_message["Encoder1"] = self.encoder_message["Encoder2"]
        self.encoder_message["Encoder2"] = temp

    def generate_messages_periodic(self):
        if debug_encoders:
            print(self.encoder_message)
        return self.encoder_message


class JointAngles(Topic):
    def __init__(self, topic_name="JointAngles", is_sim=False):
        super().__init__(topic_name, is_sim)
        self.angle_message = {"theta1": 0, "theta2": np.pi}

    def subscriber_periodic(self):
        joint1_counts, joint2_counts = self.messages["Encoders"].message["Encoder1"], self.messages["Encoders"].message[
            "Encoder2"]
        joint1_relative_orientation = encoder_counts_to_radians(joint1_counts)
        joint2_relative_orientation = encoder_counts_to_radians(joint2_counts)
        joint1_absolute = 0 - joint1_relative_orientation
        joint2_absolute = np.pi + joint2_relative_orientation

        self.angle_message["theta1"] = joint1_absolute
        self.angle_message["theta2"] = joint1_absolute + joint2_absolute

    def generate_messages_periodic(self):
        if debug_arm_angles:
            print(self.angle_message)
        return self.angle_message


class Arm1EndPosition(Topic):
    def __init__(self, topic_name="Arm1_end_pos", is_sim=False):
        super().__init__(topic_name, is_sim)
        self.SE3_pos = solve_forward_kinematics_joint1(0)

    def subscriber_periodic(self):
        theta1 = self.messages["JointAngles"].message["theta1"]
        # 1. Compute the joint1 position
        self.SE3_pos = solve_forward_kinematics_joint1(theta1)
        # 2. Adjust for the offset
        self.offset_pose = SE3.from_euler_and_translation(0, 0, 0, 0.04, 0, 0)
        self.SE3_pos = self.SE3_pos * self.offset_pose
        # 3. Apply the yaw rotation
        self.rotation_angle = 0  # self.messages["IMU_ORIENTATION"].message["Z"]
        self.center_of_rotation = SE3.from_euler_and_translation(0, 0, self.rotation_angle, 0, 0, 0)
        self.SE3_pos = self.center_of_rotation * self.SE3_pos

    def generate_messages_periodic(self):
        message_dict = self.SE3_pos.to_message_dict()
        return message_dict


class ArmEndPosition(Topic):
    def __init__(self, topic_name="Arm_End_Pos_SE3", is_sim=False):
        super().__init__(topic_name, is_sim)
        self.offset_pose = SE3.from_euler_and_translation(0, 0, 0, 0, 0, 0)
        self.center_of_rotation = SE3.from_euler_and_translation(0, 0, 0, 0, 0, 0)
        self.rotation_angle = np.pi
        self.SE3_end_pos = solve_forward_kinematics(0, np.pi)

    def subscriber_periodic(self):
        theta1, theta2 = self.messages["JointAngles"].message["theta1"], self.messages["JointAngles"].message["theta2"]
        # 1. Compute the end effector position
        self.SE3_end_pos = solve_forward_kinematics(theta1, theta2)
        # 2. Apply the base rotation (yaw) to the position
        self.rotation_angle = -(self.messages["IMU_ORIENTATION"].message["Z"]) + np.pi
        self.center_of_rotation = SE3.from_euler_and_translation(0, 0, self.rotation_angle, 0, 0, 0)
        self.SE3_end_pos = self.center_of_rotation * self.SE3_end_pos
        # 3. Adjust for the offset
        self.offset_pose = SE3.from_euler_and_translation(0, 0, 0, 0.04, 0, 0)
        self.SE3_end_pos = self.SE3_end_pos * self.offset_pose

    def generate_messages_periodic(self):
        message_dict = self.SE3_end_pos.to_message_dict()
        return message_dict


class ScannerPosition(Topic):
    def __init__(self, topic_name="Scanner_Position_SE3", is_sim=False):
        super().__init__(topic_name, is_sim)
        self.scanner_SE3 = SE3.from_euler_and_translation(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def subscriber_periodic(self):
        # Transformation of arm's end effector
        arm_end_SE3 = SE3.from_message_dict(self.messages["Arm_End_Pos_SE3"].message)
        # Orientation of the scanner from its IMU

        orientation_of_scanner = SO3.from_message_dictQuaternion(
            self.messages["SCANNER_ORIENTATION"].message
        )
        # The center of rotation takes the position from arm_end_SE3 but the orientation from the IMU
        center_of_rotation = SE3(orientation_of_scanner, arm_end_SE3.translation)
        # Relative position of the scanner to the center of rotation
        scanner_relative_position = SE3(SO3(np.eye(3)), np.array(
            [scanner_x_rel, scanner_y_rel, scanner_z_rel]))  # Assuming no change in orientation
        # Compute the scanner's absolute transformation
        self.scanner_SE3 = center_of_rotation * scanner_relative_position

    def generate_messages_periodic(self):
        message_dict = self.scanner_SE3.to_message_dict()
        if show_final_pos:
            print(message_dict)
        return message_dict


class GPIO_pin_topic(Topic):
    def __init__(self, topic_name="GPIO_pin_read", is_sim=False):
        super().__init__(topic_name, is_sim)
        self.replace_message_with_log = True

    def generate_messages_periodic(self):
        message = {"pin": GPIO.input(14)}
        if debug_gpio_pin:
            print(message)
        return message


class GPIO_printer(Topic):
    def __init__(self, subscriber_name="GPIOTrigger", is_sim=False):
        super().__init__(subscriber_name, is_sim)
        self.counter = 0

        self.previous_val = 0
        self.data_to_print = None
        self.output_dict = {}

    def subscriber_periodic(self):
        current_val = self.messages["GPIO_pin_read"].message["pin"]
        rising = current_val == 1 and self.previous_val == 0
        self.previous_val = current_val
        if rising:
            self.counter += 1
            self.output_dict = dict(self.messages["Scanner_Position_SE3"].message)
            self.output_dict["acquisition"] = self.counter
            if print_on_gpio_output:
                print(self.output_dict)
        else:
            self.output_dict = {}

    def generate_messages_periodic(self):
        return self.output_dict


class ZeroScannerIMUCommand(Command):
    def __init__(self, scanner_imu_topic):
        super().__init__([scanner_imu_topic])
        self.scanner_imu = scanner_imu_topic

    def first_run_behavior(self):
        msg = self.scanner_imu.imu_message
        x, y, z = msg["X"], msg["Y"], msg["Z"]
        print("zero imu")
        self.scanner_imu.x_offset = x
        self.scanner_imu.y_offset = y
        self.scanner_imu.z_offset = z

    def periodic(self):
        pass

    def is_complete(self):
        return True


class ZeroEncoderCommand(Command):
    def __init__(self, arm_encoder_topic):
        super().__init__([arm_encoder_topic])
        self.arm_encoder_topic = arm_encoder_topic

    def first_run_behavior(self):
        msg = self.arm_encoder_topic.encoder_message
        encoder1 = msg["Encoder1"]
        encoder2 = msg["Encoder2"]
        print("zero with " + str(encoder1))
        self.arm_encoder_topic.zero_offset_1 = encoder1
        self.arm_encoder_topic.zero_offset_2 = encoder2

    def periodic(self):
        pass

    def is_complete(self):
        return True


if __name__ == "__main__":
    scheduler = Scheduler(False, "trial_3_data copy.csv", True)
    scheduler.num_runs_per_transmission = 20
    serial_reader = SerialBusReader(is_sim=not is_rpi)
    imu = IMU()
    imu_scanner = Scanner_IMU()
    encoders = Encoders()
    joint_angles = JointAngles()
    arm_1_pos = Arm1EndPosition()
    arm_end_pos = ArmEndPosition()
    scanner_pos = ScannerPosition()
    serial_reader.add_subscriber(imu)
    serial_reader.add_subscriber(encoders)
    serial_reader.add_subscriber(imu_scanner)
    encoders.add_subscriber(joint_angles)
    joint_angles.add_subscriber(arm_end_pos)
    joint_angles.add_subscriber(arm_1_pos)
    imu.add_subscriber(arm_end_pos)
    imu.add_subscriber(arm_1_pos)
    imu.add_subscriber(scanner_pos)
    imu_scanner.add_subscriber(scanner_pos)
    arm_end_pos.add_subscriber(scanner_pos)
    joint_angles.add_subscriber(scanner_pos)

    gpio_pin = GPIO_pin_topic()

    gpio_printing = GPIO_printer()

    gpio_pin.add_subscriber(gpio_printing)
    scanner_pos.add_subscriber(gpio_printing)

    zero_command = DelayCommand(0.3, scheduler.sysTimeTopic) \
        .setNext(ZeroEncoderCommand(encoders))
    scheduler.add_topics(serial_reader, imu, joint_angles, arm_end_pos, imu_scanner, scanner_pos, gpio_pin,
                         gpio_printing)
    command = zero_command
    command.setNext(ZeroScannerIMUCommand(imu_scanner))
    scheduler.set_command_group(command)
    scheduler.initialize()

    while True:
        scheduler.periodic()
