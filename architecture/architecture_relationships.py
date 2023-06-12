from abc import ABC,abstractmethod
import time
import json


class Subscriber(ABC):
    '''
    These could be different components or subsystems of your robot. 
    For example, you might have a MotorController class that's responsible for driving the robot's motors. 
    This class could subscribe to topics that provide it with new motor commands.
    '''
    def __init__(self, is_sim, subscriber_name="Abstract Subscriber"):
        super().__init__()
        # name of topic to message pair 
        self.messages = {}
        self.is_sim = is_sim
        self.name = subscriber_name

    def initialize_hardware(self) -> bool:
        '''
        initialize hardware if applicable.  

        Returns True if all is well (no hardware is being checked or all hardware has successfully checked)
        Returns False if there is any hardware failure.  

        '''
        return True
    
    def periodic(self):
        '''
        Executes periodic tasks for the subscriber, using a different method for simulation mode
        '''
        if self.is_sim:
            self.subscriber_periodic_sim()
        else:
            self.subscriber_periodic()


    @abstractmethod
    def subscriber_periodic(self):
        '''
        after we have stored all the messages from our topics, what do we want to do with this information

        This is for you to implement  
        '''
        pass

    def subscriber_periodic_sim(self):
        '''
        optional simulation method that one can override if they want different logic than their noirmal periodic
        '''
        self.subscriber_periodic()

    def store_messages(self, topic_name: str, message: 'Message'):
        self.messages[topic_name] = message


class Message:
    '''
    a message is simply a dictionary and a time stamp of when that dictionary was created. 

    Each Topic will know what dictionary elements need to be modified for each message. 
    '''
    def __init__(self, message: dict):
        self.message = message
        if type(message) != dict:
            raise TypeError(f"Message must be a dictionary instead of {type(message)}")
        self.time_stamp = time.time()

    def __str__(self) -> str:
        return f"Message: {json.dumps(self.message)} at time {self.time_stamp}"

    def __repr__(self) -> str:
        return self.__str__()

class Command(ABC):
    def __init__(self, subscribers: list):
        self.dependent_subscribers = subscribers
        self.next_command = None
        self.first_run_occured = False

    def first_run(self):
        if not self.first_run_occured:
            self.first_run_behavior()
            self.first_run_occured = True

    @abstractmethod
    def first_run_behavior(self):
        pass

    @abstractmethod
    def periodic(self):
        pass

    @abstractmethod
    def is_complete(self) -> bool:
        return False

    def setNext(self, next_command: 'Command'):
        """
        Set the next command to be executed once this command is complete.
        If this command already has a next command, append the new command to the end of the chain.
        """
        if isinstance(next_command, Command):
            if self.next_command is None:
                self.next_command = next_command
            else:
                last_command = self.next_command
                while last_command.next_command is not None:
                    last_command = last_command.next_command
                last_command.next_command = next_command
        else:
            raise TypeError("next_command must be an instance of Command")


class ParallelCommand(Command):
    def __init__(self, commands, name="Parallel Command"):
        # The list of commands to run in parallel
        self.commands = commands
        self.name = name
        self.first_run_occured = False

    def first_run_behavior(self):
        for command in self.commands:
            command.first_run()
        self.first_run_occured = True

    def periodic(self):
        for i, command in enumerate(self.commands):
            if command.is_complete() and command.next_command:
                # Move to the next command if the current one is complete
                self.commands[i] = command.next_command
                self.commands[i].first_run()
            else:
                command.periodic()

    def is_complete(self):
        # Returns True only if all commands have completed
        return all(command.is_complete() for command in self.commands)
    

class Topic(Subscriber):
    '''
    These could be various sensor readings or commands. 
    For example, you might have a SpeedCommand topic that the MotorController subscribes to. 
    Whenever a new message is published on this topic, 
    the MotorController would update the robot's speed accordingly. 
    Similarly, sensor topics can publish data 
    such as images from a camera, distance from an ultrasonic sensor, etc.
    '''
    def __init__(self, name= "Abstract Topic", is_sim=False):
        super().__init__(is_sim, name)
        self.subscribers = []
        self.message_body = {}
        self.__current_time = time.time()
        self.__previous_time = time.time()
        self.delta_time_seconds = self.__current_time - self.__previous_time
        # when true, if we are simulating, we will publish the message that was in the log file instead of generating a new one
        # important if this topic is just used for reading data from a physical sensor that the simulation does not have access to read.  
        # should be kept false if the topic can be simulated or is calculated based on data from other topics.  Odometry and PID for example SHOULD be simulated
        self.replace_message_with_log = False


    def subscriber_periodic(self):
        pass

    @abstractmethod
    def generate_messages_periodic(self):
        '''
        
        Topics are things like speeds a motor controller needs to reach or velocity estimated from an encoder.  

        here is where you define how your messages are formatted.  
        
        '''
        pass

    def publish_periodic(self) -> Message:
        self.__current_time = time.time()
        self.delta_time_seconds = self.__current_time - self.__previous_time
        self.message_body = self.generate_messages_periodic()
        self.__previous_time = self.__current_time 
        msg = Message(self.message_body)
        self.notify_subscribers(msg)
        return msg, self.__current_time, self.delta_time_seconds
    
    def publish_periodic_from_log(self, message_from_log: 'Message', current_time, delta_time_seconds) -> Message:
        self.__current_time = current_time
        self.delta_time_seconds = delta_time_seconds
        self.__previous_time = self.__current_time
        self.notify_subscribers(message_from_log)
        return message_from_log, self.__current_time, self.delta_time_seconds
    

    def notify_subscribers(self, msg: Message):
        for sub in self.subscribers:
            sub.store_messages(self.name, msg)

    def add_subscriber(self, sub: Subscriber):
        self.subscribers.append(sub)

    def __str__(self) -> str:
        return f"Topic: {self.name} with message {self.message_body}"
