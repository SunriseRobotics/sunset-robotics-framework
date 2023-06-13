from architecture.architecture_relationships import *
from architecture.scheduler import *
# -*- coding: future_fstrings -*-

class Motor(Subscriber):
    def __init__(self, subscriber_name="Motor",is_sim=False):
        super().__init__(subscriber_name,is_sim)
        self.voltage_hardware = 0
    def subscriber_periodic(self):
        print(f"messages {self.messages}")
        self.voltage_hardware = self.messages["VelocityPID"].message["power"]
    def __str__(self) -> str:
        return f"Motor Voltage Topic(Voltage: {self.voltage_hardware})"

    def __repr__(self) -> str:
        return self.__str__()

class VelocityPID(Topic):
    def __init__(self,topic_name="VelocityPID",is_sim=False):
        super().__init__(topic_name,is_sim)
        self.reference_velocity = 0 
        self.estimated_velocity = 0 
        self.power_message = 0
    def generate_messages_periodic(self):
        print(self.messages)
        self.reference_velocity = self.messages["ReferenceVelocity"].message["reference_velocity"]
        self.estimated_velocity = self.messages["MotorVelocity"].message["motor_velocity"]
        self.power_message = 0.1 * (self.reference_velocity - self.estimated_velocity)    
        return {"power": self.power_message}
    def __str__(self) -> str:
        return f"VelocityPID Topic(Power: {self.power_message})"

    def __repr__(self) -> str:
        return self.__str__()
    
class MotorVelocity(Topic):
    def __init__(self,topic_name="MotorVelocity",is_sim=False):
        super().__init__(topic_name,is_sim)
        self.replace_message_with_log = True

    def generate_messages_periodic(self):
        return {"motor_velocity": 1}

    def __str__(self) -> str:
        return f"MotorVelocity Topic(Velocity: 1)"

    def __repr__(self) -> str:
        return self.__str__()

class ReferenceVelocity(Topic):
    def __init__(self, topic_name="ReferenceVelocity", reference_velocity=0,is_sim=False):
        super().__init__(topic_name,is_sim)
        self.reference_velocity = reference_velocity
    def generate_messages_periodic(self):
        return {"reference_velocity": self.reference_velocity}
    def __str__(self) -> str:
        return f"ReferenceVelocity Topic(Reference Velocity: {self.reference_velocity})"

    def __repr__(self) -> str:
        return self.__str__()    

class CommandVelocity(Command):
    def __init__(self, reference_topic: 'ReferenceVelocity', reference_velocity=0):
        super().__init__([])
        self.reference_velocity = reference_velocity
        self.reference_topic = reference_topic
    def first_run_behavior(self):
        self.reference_topic.reference_velocity = self.reference_velocity
    def periodic(self):
        pass
    def is_complete(self):
        return True  
    

