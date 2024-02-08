from architecture.scheduler import Scheduler
from architecture.architecture_relationships import Command, DelayCommand, Topic, Subscriber, Message
import numpy as np
from sunset_math.geometry import SE3, SO3
from sunset_math.mobile_kinematics import MecanumWheelSpeeds, MecanumKinematics
from sunset_math.TrapezoidProfile import *
from sunset_math.graph_theory import generate_network_graph
import math

# test
TICKS_PER_REVOLUTION = 28 * 13.7
ROBOCLAW_ADDRESS_1 = 0x80
ROBOCLAW_ADDRESS_2 = 0x81
BAUD = 115200
WHEEL_DIAMETER_M = 0.096
WHEEL_C = WHEEL_DIAMETER_M * np.pi
TRACK_WIDTH_M = 0.27
kinematics = MecanumKinematics(TRACK_WIDTH_M)

debug_odom = True


def ticks_to_meters(ticks):
    revolutions = ticks / TICKS_PER_REVOLUTION
    return revolutions * WHEEL_C


def meters_to_ticks(meters):
    revolutions = meters / WHEEL_C
    return revolutions * TICKS_PER_REVOLUTION

def clip_output(val, min, max):
    if val < min:
        return min
    if val > max:
        return max
    return val

class MotorVelocity(Topic):
    def __init__(self, roboclaw_instance1, roboclaw_instance2, topic_name="MotorVelocity", is_sim=False):
        super().__init__(topic_name, is_sim)
        self.message = {"frontleft": 0, "frontright": 0, "backleft": 0, "backright": 0}
        self.replace_message_with_log = True
        self.roboclaw_instance1 = roboclaw_instance1
        self.roboclaw_instance2 = roboclaw_instance2

    def generate_messages_periodic(self):
        self.message["frontleft"] = self.roboclaw_instance2.ReadSpeedM1(ROBOCLAW_ADDRESS_2)[1]
        self.message["frontright"] = self.roboclaw_instance2.ReadSpeedM2(ROBOCLAW_ADDRESS_2)[1]
        self.message["backleft"] = self.roboclaw_instance1.ReadSpeedM2(ROBOCLAW_ADDRESS_1)[1]
        self.message["backright"] = self.roboclaw_instance1.ReadSpeedM1(ROBOCLAW_ADDRESS_1)[1]
        return self.message


class MotorPosition(Topic):
    def __init__(self, roboclaw_instance1, roboclaw_instance2, topic_name="MotorPosition", is_sim=False):
        super().__init__(topic_name, is_sim)
        self.message = {"frontleft": 0, "frontright": 0, "backleft": 0, "backright": 0}
        self.replace_message_with_log = True
        self.roboclaw_instance1 = roboclaw_instance1
        self.roboclaw_instance2 = roboclaw_instance2

    def generate_messages_periodic(self):
        self.message["frontleft"] = self.roboclaw_instance2.ReadEncM1(ROBOCLAW_ADDRESS_2)[1]
        self.message["frontright"] = self.roboclaw_instance2.ReadEncM2(ROBOCLAW_ADDRESS_2)[1]
        self.message["backleft"] = self.roboclaw_instance1.ReadEncM2(ROBOCLAW_ADDRESS_1)[1]
        self.message["backright"] = self.roboclaw_instance1.ReadEncM1(ROBOCLAW_ADDRESS_1)[1]
        return self.message


class RobotRelativeVelocity(Topic):
    def __init__(self, topic_name="RobotRelativeVelocity", is_sim=False):
        super().__init__(topic_name, is_sim)
        self.message = {"x": 0.0, "y": 0.0, "omega": 0.0}

    def subscriber_periodic(self):
        motor_velocity_message = self.messages["MotorVelocity"]
        fl = motor_velocity_message.message["frontleft"]
        fr = motor_velocity_message.message["frontright"]
        bl = motor_velocity_message.message["backleft"]
        br = motor_velocity_message.message["backright"]
        fl = ticks_to_meters(fl)
        fr = ticks_to_meters(fr)
        bl = ticks_to_meters(bl)
        br = ticks_to_meters(br)
        wheel_speeds = MecanumWheelSpeeds(fl, fr, bl, br)
        twist = kinematics.wheelVelocityToTwistRobot(wheel_speeds)
        self.message["x"] = twist[0]
        self.message["y"] = twist[1]
        self.message["omega"] = twist[5]

    def generate_messages_periodic(self):
        return self.message


class Odometry(Topic):
    def __init__(self, topic_name="Odometry_POSE2D", is_sim=False):
        super().__init__(topic_name, is_sim)
        self.pose = SE3.from_euler_and_translation(0, 0, 0, 0, 0, 0)
        self.message = {"X": 0.0, "Y": 0.0, "THETA": 0.0}

    def subscriber_periodic(self):
        dt = self.messages["SystemTime"].message["DeltaTimeSeconds"]
        velocity = self.messages["RobotRelativeVelocity"].message
        twist = SE3.from_angular_and_linear_velocities([velocity["x"] * dt, velocity["y"] * dt, 0], 0, 0
                                                       , velocity["omega"] * dt)
        self.pose *= twist

    def generate_messages_periodic(self):
        self.message["X"] = float(self.pose.translation[0])
        self.message["Y"] = float(self.pose.translation[1])
        self.message["THETA"] = float(self.pose.rotation.to_euler()[2])
        if debug_odom:
            print(self.message)
        return self.message


class RoboclawWriter(Subscriber):
    def __init__(self, roboclaw_instance1=None, roboclaw_instance2=None, topic_name="Roboclaw1", is_sim=False):
        super().__init__(is_sim, topic_name)
        self.roboclaw_instance1 = roboclaw_instance1
        self.roboclaw_instance2 = roboclaw_instance2

    def initialize_hardware(self) -> bool:
        return True

    def setVelocity(self, frontleft, frontright, backleft, backright):
        if self.is_sim:
            frontleft, frontright, backleft, backright = 0, 0, 0, 0

        frontleft = meters_to_ticks(frontleft)
        frontright = meters_to_ticks(frontright)
        backleft = meters_to_ticks(backleft)
        backright = meters_to_ticks(backright)

        self.roboclaw_instance2.SpeedM1(ROBOCLAW_ADDRESS_2, int(frontleft))
        self.roboclaw_instance2.SpeedM2(ROBOCLAW_ADDRESS_2, int(frontright))
        self.roboclaw_instance1.SpeedM2(ROBOCLAW_ADDRESS_1, int(backleft))
        self.roboclaw_instance1.SpeedM1(ROBOCLAW_ADDRESS_1, int(backright))

    def subscriber_periodic(self):
        pass


class setMotorVelocityCommand(Command):
    def __init__(self, motorController: 'RoboclawWriter', frontleft, frontright, backleft, backright):
        super().__init__([])
        self.frontleft = frontleft
        self.frontright = frontright
        self.backleft = backleft
        self.backright = backright
        self.motorController = motorController

    def first_run_behavior(self):
        self.motorController.setVelocity(self.frontleft, self.frontright, self.backleft, self.backright)

    def periodic(self):
        pass

    def is_complete(self):
        return True


TAU = math.pi * 2


def angle_wrap(angle):
    while angle < -math.pi:
        angle += 2.0 * math.pi
    while angle > math.pi:
        angle -= 2.0 * math.pi
    return angle


def normalize_angle_rr(radians):
    modified_angle = radians % TAU
    modified_angle = (modified_angle + TAU) % TAU
    return modified_angle


def normalize_angle(radians):
    return angle_wrap(-normalize_angle_rr(radians))


def normalized_heading_error(reference_radians, state_radians):
    return normalize_angle(reference_radians - state_radians)


class goToPoseMotionProfile(Command):
    def __init__(self, motorController: 'RoboclawWriter', odometry, x, y, theta, timer):
        super().__init__([motorController, odometry])
        self.x_target = x
        self.y_target = y
        self.theta_target = theta
        self.motor_controller = motorController
        self.odometry = odometry
        self.distance = 10
        self.distance_cutoff = 0.08
        self.heading_error = 10
        self.heading_cutoff_rad = np.radians(1)
        self.max_velo = 0.5  # m/s
        self.max_accel = 0.5  # m/s^2
        self.profileX = None
        self.profileY = None
        self.initial_x = 0
        self.initial_y = 0
        self.distance_x = 0
        self.distance_y = 0
        self.timer = timer
        self.initial_time = 0

    def first_run_behavior(self):
        odometry_message = self.odometry.message
        global_x = odometry_message["X"]
        global_y = odometry_message["Y"]
        self.initial_x = global_x
        self.initial_y = global_y
        self.distance_x = self.x_target - self.initial_x
        self.distance_y = self.y_target - self.initial_y
        print(f"initializing motion profile, x dist : {self.distance_x}, y dist: {self.distance_y}")
        self.profileX = TrapezoidProfile(self.max_accel, self.max_velo, -self.max_accel, self.distance_x)
        self.profileY = TrapezoidProfile(self.max_accel, self.max_velo, -self.max_accel, self.distance_y)
        self.initial_time = self.timer.message["Unix"]

    def periodic(self):
        odometry_message = self.odometry.message
        global_x = odometry_message["X"]
        global_y = odometry_message["Y"]
        global_theta = odometry_message["THETA"]

        global_x_error = self.x_target - global_x
        global_y_error = self.y_target - global_y
        global_theta_error = normalized_heading_error(self.theta_target, global_theta)
        self.heading_error = global_theta_error
        self.distance = np.hypot(global_x_error, global_y_error)

        # for the errors we track we want to get them from the motion profile
        time_since = self.timer.message["Unix"] - self.initial_time
        global_error_x = self.initial_x + self.profileX.getState(time_since).x - global_x
        # negate this because I always have, and it breaks if I don't
        global_error_y = -(self.initial_y + self.profileY.getState(time_since).x - global_y)

        print(global_error_x)
        # Rotate the error to local coordinates
        local_error_x = global_error_x * np.cos(global_theta) - global_error_y * np.sin(global_theta)
        local_error_y = global_error_x * np.sin(global_theta) + global_error_y * np.cos(global_theta)

        x_power = local_error_x * 1.5
        y_power = local_error_y * 1.5
        turn_power = clip_output(global_theta_error * 2, -0.25, 0.25)
        # print(str(x_power) + " " + str(y_power) + " " + str(turn_power))

        x = x_power
        y = y_power
        fl = x + y + turn_power
        bl = x - y + turn_power
        fr = x - y - turn_power
        br = x + y - turn_power

        self.motor_controller.setVelocity(fl, fr, bl, br)

    def is_complete(self):
        return self.distance < self.distance_cutoff and math.fabs(self.heading_error) < self.heading_cutoff_rad


class goToPose(Command):
    def __init__(self, motorController: 'RoboclawWriter', odometry, x, y, theta):
        super().__init__([motorController, odometry])
        self.x_target = x
        self.y_target = y
        self.theta_target = theta
        self.motor_controller = motorController
        self.odometry = odometry
        self.distance = 10
        self.distance_cutoff = 0.08
        self.heading_error = 10
        self.heading_cutoff_rad = np.radians(1)

    def first_run_behavior(self):
        pass

    def periodic(self):
        odometry_message = self.odometry.message
        global_x = odometry_message["X"]
        global_y = odometry_message["Y"]
        global_theta = odometry_message["THETA"]

        global_x_error = self.x_target - global_x
        global_y_error = self.y_target - global_y
        global_theta_error = normalized_heading_error(self.theta_target, global_theta)
        self.heading_error = global_theta_error
        self.distance = np.hypot(global_x_error, global_y_error)
        global_error_x = self.x_target - global_x
        # negate this because I always have, and it breaks if i don't
        global_error_y = -(self.y_target - global_y)

        # Rotate the error to local coordinates
        local_error_x = global_error_x * np.cos(global_theta) - global_error_y * np.sin(global_theta)
        local_error_y = global_error_x * np.sin(global_theta) + global_error_y * np.cos(global_theta)

        x_power = local_error_x * 1.5
        y_power = local_error_y * 1.5
        turn_power = global_theta_error * 2
        print(str(x_power) + " " + str(y_power) + " " + str(turn_power))

        x = x_power
        y = y_power
        fl = x + y + turn_power
        bl = x - y + turn_power
        fr = x - y - turn_power
        br = x + y - turn_power

        self.motor_controller.setVelocity(fl, fr, bl, br)

    def is_complete(self):
        return self.distance < self.distance_cutoff and math.fabs(self.heading_error) < self.heading_cutoff_rad


def main():
    is_running_on_robot = False
    try:
        from roboclaw import Roboclaw
        roboclaw1 = Roboclaw('/dev/ttyACM1', BAUD)
        roboclaw2 = Roboclaw('/dev/ttyACM0', BAUD)
        roboclaw1.Open()
        roboclaw2.Open()
        roboclaw1.ResetEncoders(ROBOCLAW_ADDRESS_1)
        roboclaw2.ResetEncoders(ROBOCLAW_ADDRESS_2)
    except:
        is_running_on_robot = False
        roboclaw1 = None
        roboclaw2 = None

    scheduler = Scheduler(False, "log_1688448006.csv", enable_coms=False)
    roboclaw = RoboclawWriter(roboclaw1, roboclaw2, is_sim=scheduler.is_sim)
    motor_pos_topic = MotorPosition(roboclaw1, roboclaw2)
    motor_velo_topic = MotorVelocity(roboclaw1, roboclaw2)
    robot_velo_topic = RobotRelativeVelocity()
    odom_topic = Odometry()

    motor_velo_topic.add_subscriber(robot_velo_topic)
    robot_velo_topic.add_subscriber(odom_topic)
    scheduler.sysTimeTopic.add_subscriber(odom_topic)

    scheduler.add_topics(motor_pos_topic, motor_velo_topic)
    scheduler.add_subscribers(roboclaw)

    def generate_reliable_gtp(x, y, theta_rad):
        # forces the velocity to go to zero at the end of a follow
        c = goToPose(roboclaw, odom_topic, x, y, theta_rad)
        c.setNext(setMotorVelocityCommand(roboclaw, 0, 0, 0, 0))
        c.setNext(DelayCommand(0.25, scheduler.sysTimeTopic))
        return c

    def generate_mp_gtp(x, y, theta_rad):
        c = goToPoseMotionProfile(roboclaw, odom_topic, x, y, theta_rad, scheduler.sysTimeTopic)
        c.setNext(setMotorVelocityCommand(roboclaw, 0, 0, 0, 0))
        c.setNext(DelayCommand(0.25, scheduler.sysTimeTopic))
        return c

    command = generate_mp_gtp(0, 1, np.radians(120))
    command.setNext(generate_mp_gtp(0,0,np.radians(180)))
    command.setNext(generate_mp_gtp(1, 0, np.radians(90)))
    command.setNext(generate_mp_gtp(0, 0, np.radians(-15)))

    command.setNext(setMotorVelocityCommand(roboclaw, 0.0, 0.0, 0.0, 0.0))

    scheduler.set_command_group(command)
    scheduler.initialize()

    if not is_running_on_robot:
        generate_network_graph(scheduler.topics, scheduler.subscribers)
        return
    running = True
    while running:
        try:
            scheduler.periodic()
        except Exception as e:
            print(e)
            running = False
            roboclaw.setVelocity(0, 0, 0, 0)
            return


if __name__ == "__main__":
    main()
