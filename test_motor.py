from example.MotorControllerExample.MotorHardware import CommandVelocity, Motor, MotorVelocity, ReferenceVelocity, VelocityPID
from architecture.scheduler import Scheduler


if __name__ == "__main__":
    motor = Motor(is_sim = False)
    velocity_pid = VelocityPID(is_sim =False)
    motor_velocity = MotorVelocity(is_sim =False)
    reference_velocity = ReferenceVelocity(is_sim =False, reference_velocity=1)
    # set relationships
    reference_velocity.add_subscriber(velocity_pid)
    motor_velocity.add_subscriber(velocity_pid)
    velocity_pid.add_subscriber(motor)
    # set commands
    command_velocity = CommandVelocity(reference_velocity, reference_velocity=1)
    # set scheduler
    scheduler = Scheduler(True, "log_1686678370.csv")
    
    scheduler.set_command_group(command_velocity)
    scheduler.add_subscribers(motor)
    scheduler.add_topics(reference_velocity, motor_velocity, velocity_pid)
    scheduler.initialize()
    # run schedule3r
    for i in range(1000):
        scheduler.periodic()
        print("Motor Voltage: {}".format(motor.voltage_hardware))
        print("Reference Velocity: {}".format(reference_velocity.reference_velocity))
    

