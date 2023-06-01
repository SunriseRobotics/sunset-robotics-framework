from architecture.architecture_relationships import *
from architecture.scheduler import *

class Motor(Subscriber):
    def __init__(self, is_sim, subscriber_name="Motor"):
        super().__init__(is_sim, subscriber_name)
        self.voltage_hardware = 0
    def subscriber_periodic(self):
        self.voltage_hardware = self.messages["Power"].message["power"]

class VelocityPID(Topic):
    def __init__(self, is_sim, topic_name="VelocityPID"):
        super().__init__(is_sim, topic_name)
        self.reference_velocity = 0 
        self.estimated_velocity = 0 
        self.power_message = 0
    def generate_messages_periodic(self):
        return {"power": self.power_message}
    def subscriber_periodic(self):
        self.reference_velocity = self.messages["ReferenceVelocity"].message["reference_velocity"]
        self.estimated_velocity = self.messages["MotorVelocity"].message["motor_velocity"]
        self.power_message = 0.1 * (self.reference_velocity - self.estimated_velocity)    

class MotorVelocity(Topic):
    def __init__(self, is_sim, topic_name="MotorVelocity"):
        super().__init__(is_sim, topic_name)

    def generate_messages_periodic(self):
        return {"motor_velocity": 0}
    

class ReferenceVelocity(Topic):
    def __init__(self, is_sim, topic_name="ReferenceVelocity", reference_velocity=0):
        super().__init__(is_sim, topic_name)
        self.reference_velocity = reference_velocity
    def generate_messages_periodic(self):
        return {"reference_velocity": self.reference_velocity}


class CommandVelocity(Command):
    def __init__(self, is_sim, reference_topic: 'ReferenceVelocity', reference_velocity=0):
        super().__init__(is_sim)
        self.reference_velocity = reference_velocity
        self.reference_topic = reference_topic
    def first_run_behavior(self):
        self.reference_topic.reference_velocity = self.reference_velocity
    def periodic(self):
        pass
    def is_complete(self):
        return True  
    

