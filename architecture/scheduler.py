from architecture.architecture_relationships import Command, Subscriber, Topic, Message
from pyrose_math.graph_theory import dependency_sort, cycle_is_present_in_any
from pyrose_exceptions.pyros_exceptions import TopicCircularDependency, TopicNameCollision, SubscriberNameCollision
from architecture.OnRobotUDP import start_client, send_data_to_server
import time
import csv
import json

import architecture.topicLogUtil as topicLogUtil


class Scheduler:
    def __init__(self, is_sim=False, file_reading_name=None, enable_coms=False):
        self.topics = []
        self.subscribers = []
        self.is_sim = is_sim
        self.throw_exception_on_init_failure = True
        self.root_command = None
        self.writing_file_name = None
        self.file_reading_name = file_reading_name
        self.read_topics = None
        self.time_stamps = None
        self.has_initialize_been_called = False
        self.enable_coms = enable_coms
        self.client_socket = None
        self.server_address = None
        self.num_runs_per_transmission = 100
        self.num_runs = 0

    def initialize(self):

        self.has_initialize_been_called = True

        self.topics = dependency_sort(self.topics)

        if cycle_is_present_in_any(self.topics):
            raise TopicCircularDependency("There is a circular dependency in the topics, aborting init")
        for topic in self.topics:
            print("Topic name: {}".format(topic.name))

        print("Beginning log...")
        self.begin_log()
        print("Initializing hardware...")
        self.init_hardware()
        print("finished initializing hardware...")
        self.check_topic_name_collision()
        print("name check finished...")

        if self.is_sim:
            if self.file_reading_name is None:
                raise Exception("Must provide file name to read from in simulation mode.")
            self.time_stamps, messages_contents = topicLogUtil.dump_file_contents(self.file_reading_name)
            self.read_topics = topicLogUtil.construct_dictionary_of_messages_vs_time(self.time_stamps,
                                                                                     messages_contents)

        if self.enable_coms:
            self.client_socket, self.server_address = start_client()
            print("UDP started on client side: {} {}".format(self.client_socket, self.server_address))

    def advance_command(self):
        if self.root_command and self.root_command.is_complete():
            self.root_command = self.root_command.next_command

    def periodic(self):

        if not self.has_initialize_been_called:
            raise Exception(
                "Must call initialize before calling periodic otherwise hardware devices will not be connected...")

        stored_messages = {}
        present_time = time.time()
        if self.is_sim:
            try:
                present_time = self.time_stamps.pop(0)
            except IndexError:
                print("Simulation is over, no more messages to read")
                return
        if not (self.root_command is None) and not self.root_command.first_run_occurred:
            self.root_command.first_run()

        self.advance_command()

        # Check if root_command is still not None after advancing
        if self.root_command is not None:
            self.root_command.periodic()
        else:
            print("No further command to execute")

        all_logged_messages = None

        if self.is_sim:
            all_logged_messages = topicLogUtil.get_message_at_time(present_time, self.read_topics)

        for topic in self.topics:
            if self.is_sim and topic.replace_message_with_log:
                message_dictionary, current_time_seconds, delta_time_seconds = all_logged_messages[topic.name]
                message = Message(message_dictionary)
                message.time_stamp = current_time_seconds
                topic.publish_periodic_from_log(message, current_time_seconds, delta_time_seconds)
                stored_messages[topic.name] = "{0}, {1}, {2}".format(json.dumps(message.message), current_time_seconds,
                                                                     delta_time_seconds)
                # if we are simulating, and we are replacing the message with a log, then we need to get the message
                # from the log
            else:
                # if not sim, or if we are not replacing the message with a log, then we need to publish the message
                # since this implies it can be calculated with known inputs
                message, current_time_seconds, delta_time_seconds = topic.publish_periodic()
                stored_messages[topic.name] = "{0}, {1}, {2}".format(json.dumps(message.message), current_time_seconds,
                                                                     delta_time_seconds)

        stored_messages_txt = "{0}, {1}\n".format(present_time, json.dumps(stored_messages))

        # write the messages to the log file
        if not self.is_sim and stored_messages:
            with open(self.writing_file_name, 'a+') as self.f:
                self.f.write(stored_messages_txt)

        for sub in self.subscribers:
            sub.periodic()

        # often topics are also subscribers.  All topics inherit from subscriber
        for topic in self.topics:
            topic.periodic()

        if self.enable_coms:
            send_data_to_server(self.client_socket, self.server_address, stored_messages_txt)


    def set_command_group(self, head: Command):
        self.root_command = head

    def shutdown(self):
        pass

    # add all topics using args 
    def add_topics(self, *args):
        for topic in args:
            self.topics.append(topic)

    # add all subscribers using args
    def add_subscribers(self, *args):
        for sub in args:
            self.subscribers.append(sub)

    def begin_log(self):
        if not self.is_sim:
            self.writing_file_name = "log_" + str(int(time.time())) + ".csv"
            with open(self.writing_file_name, 'w') as self.f:
                self.f.write("Time, TopicMessageDictionaries\n")

    def init_hardware(self):
        for sub in self.subscribers:
            success = sub.initialize_hardware()
            if self.throw_exception_on_init_failure and not success:
                raise RuntimeError("Hardware for Subscriber, '{}' failed to initialize, aborting init".format(sub.name))
            sub.is_sim = self.is_sim

        for topic in self.topics:
            success = topic.initialize_hardware()
            if self.throw_exception_on_init_failure and not success:
                raise RuntimeError("Hardware for Topic, '{}' failed to initialize, aborting init".format(topic.name))
            topic.is_sim = self.is_sim

    def check_topic_name_collision(self):
        visited_topics = []
        for topic in self.topics:
            for topic_other in visited_topics:
                if topic.name == topic_other.name:
                    raise RuntimeError(
                        "Two or more Topics cannot have the same name: {}, Please Check your Configuration".format(
                            topic.name))

            visited_topics.append(topic)

            for sub in self.subscribers:
                if topic.name == sub.name:
                    raise RuntimeError("Topic and Subscriber cannot have the same name: {}".format(topic.name))
