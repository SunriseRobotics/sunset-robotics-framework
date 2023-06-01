from example.MotorControllerExample.MotorHardware import CommandVelocity, Motor, MotorVelocity, ReferenceVelocity, VelocityPID
from architecture.scheduler import Scheduler


if __name__ == "__main__":
    motor = Motor(False)
    velocity_pid = VelocityPID(False)
    motor_velocity = MotorVelocity(False)
    reference_velocity = ReferenceVelocity(False, reference_velocity=1)
    # set relationships
    reference_velocity.add_subscriber(velocity_pid)
    motor_velocity.add_subscriber(velocity_pid)
    velocity_pid.add_subscriber(motor)
    # set commands
    command_velocity = CommandVelocity(False, reference_velocity, reference_velocity=1)
    # set scheduler
    scheduler = Scheduler()
    scheduler.set_command_group(command_velocity)
    scheduler.initialize()
    # run scheduler
    for i in range(100):
        scheduler.periodic()
        print("Motor Voltage: {}".format(motor.voltage_hardware))
        print("Reference Velocity: {}".format(reference_velocity.reference_velocity))
    

