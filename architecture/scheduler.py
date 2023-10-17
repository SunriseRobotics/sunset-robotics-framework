from architecture.architecture_relationships import Command, Subscriber, Topic, Message, SystemTimeTopic
from sunset_math.graph_theory import dependency_sort, cycle_is_present_in_any
from architecture.OnRobotUDP import start_client, send_data_to_server
import time
import csv
import json

import architecture.topicLogUtil as topicLogUtil


class Scheduler:
    """
    The scheduler where all the magic happens,
    """
    def __init__(self, is_sim=False, file_reading_name=None, enable_coms=False):
        # system time topic field which you will use for instantiating delay commands.
        self.sysTimeTopic = SystemTimeTopic()
        # list of system topics
        self.topics = []
        # list of system subscribers
        self.subscribers = []
        # flag to determine if we are simulating from log file or running on actual hardware.
        self.is_sim = is_sim
        # if we fail to initialize, throw an exception to fail loudly.
        self.throw_exception_on_init_failure = True
        # this is where your chain of commands exists.
        self.root_command = None
        # file name we will write to
        self.writing_file_name = None
        self.file_reading_name = file_reading_name
        self.read_topics = None
        self.time_stamps = None
        self.has_initialize_been_called = False
        # determine if we want to enable wireless communication via UDP for use with external network tools.
        self.enable_coms = enable_coms
        # if applicable the client and server addresses of our UDP server.
        self.client_socket = None
        self.server_address = None
        # number of loop iterations between transmissions.  Higher value will be less taxing on network resources.
        self.num_runs_per_transmission = 100
        # counter used to determine time since the last transmission.
        self.num_runs = 0
        self.debug = False
        # why would you not log.
        self.should_log = True
        # replay rate (percentage) TODO: implement this.
        self.replay_speed = 1
        # current status of the replay.  Not actually utilized if running in run mode.
        self.replay_time = 0
        # start of the replay to offset the current unix timestamp from those found in the log.
        self.replay_start_time = 0
        # the initial unix time stamp in the log.
        self.initial_time_of_log = 0
        # if this is the first time running the simulation, there might be special cases we need to handle.
        self.first_sim_run = True

    def initialize(self):

        """
        Initialize is called to ensure that all the pyROSe systems are behaving as expected.
        """

        self.has_initialize_been_called = True

        # add the system time topic to our list of user-supplied topics.
        self.add_topics(self.sysTimeTopic)

        # determine the order we loop over the topics so that we always have fresh data.
        self.topics = dependency_sort(self.topics)

        # guarantee pyrose does not start with a circular dependency in its topics.
        if cycle_is_present_in_any(self.topics):
            raise Exception("There is a circular dependency in the topics, aborting init")
        for topic in self.topics:
            print("Topic name: {}".format(topic.name))

        print("Beginning log...")
        self.begin_log()
        print("Initializing hardware...")
        self.init_hardware()
        print("finished initializing hardware...")
        self.check_topic_name_collision()
        print("name check finished...")

        # setup file handling if we are reading from a log file.
        if self.is_sim:
            if self.file_reading_name is None:
                raise Exception("Must provide file name to read from in simulation mode.")
            self.time_stamps, messages_contents = topicLogUtil.dump_file_contents(self.file_reading_name)
            self.read_topics = topicLogUtil.construct_dictionary_of_messages_vs_time(self.time_stamps,
                                                                                     messages_contents)
            self.replay_start_time = time.time()
        # start our UDP server.
        if self.enable_coms:
            self.client_socket, self.server_address = start_client()
            print("UDP started on client side: {} {}".format(self.client_socket, self.server_address))

    def advance_command(self):
        """
        if we have a command and the current command is complete, then proceed to the next command.
        """
        if self.root_command and self.root_command.is_complete():
            self.root_command = self.root_command.next_command

    def periodic(self):
        """
        Periodic, run for the lifetime of your system.  Robustly handle everything.
        """

        # ensure we have initialized.
        if not self.has_initialize_been_called:
            raise Exception(
                "Must call initialize before calling periodic otherwise hardware devices will not be connected...")

        stored_messages = {}
        present_time = time.time()

        if self.is_sim:
            # since the unix time stamp of when the sim starts will be different
            # then the time stamp when the robot started, we must offset to a zero-based index
            self.replay_time = time.time() - self.replay_start_time
            if not self.first_sim_run:
                if present_time - self.initial_time_of_log < self.replay_time:
                    return

            try:
                # try to pop the next datum off the queue.
                present_time = self.time_stamps.pop(0)

                # if this is the firs time running use this time as a reference.
                if self.first_sim_run:
                    self.replay_start_time = present_time
                    self.first_sim_run = False
            except IndexError:
                if self.debug:
                    print("Simulation is over, no more messages to read")

        all_logged_messages = None

        if self.is_sim:
            # if sim read the messages for the current time stamp.
            all_logged_messages = topicLogUtil.get_message_at_time(present_time, self.read_topics)

        # iterate through all of our topics.
        for topic in self.topics:
            if self.is_sim and topic.replace_message_with_log:
                # if we are simulating, and we are replacing the message with a log, then we need to get the message
                # from the log
                message_dictionary, current_time_seconds, delta_time_seconds = all_logged_messages[topic.name]
                message = Message(message_dictionary)
                message.time_stamp = current_time_seconds
                topic.publish_periodic_from_log(message, current_time_seconds, delta_time_seconds)
                stored_messages[topic.name] = "{0}, {1}, {2}".format(json.dumps(message.message), current_time_seconds,
                                                                     delta_time_seconds)

            else:
                # if not sim, or if we are not replacing the message with a log, then we need to publish the message
                # since this implies it can be calculated with known inputs

                # this has a cool side effect of actually running topics, so we can test changes in the sim.
                message, current_time_seconds, delta_time_seconds = topic.publish_periodic()
                stored_messages[topic.name] = "{0}, {1}, {2}".format(json.dumps(message.message), current_time_seconds,
                                                                     delta_time_seconds)

        # take all the current messages and store them as a string.
        stored_messages_txt = "{0}, {1}\n".format(present_time, json.dumps(stored_messages))

        # write the messages to the log file
        if not self.is_sim and stored_messages and self.should_log:
            with open(self.writing_file_name, 'a+') as self.f:
                self.f.write(stored_messages_txt)

        # call the subscribers now that all of our topics have finished publishing.
        for sub in self.subscribers:
            sub.periodic()

        # Often topics are also subscribers.  All topics inherited from subscriber
        for topic in self.topics:
            topic.periodic()

        # if we have a command and have not started running it, then start!
        if not (self.root_command is None) and not self.root_command.first_run_occurred:
            self.root_command.first_run()
        elif not (self.root_command is None):
            # TODO determine if this prevents race conditions -- delete once validated.  Unit tests still pass...
            # check if the command is done,
            self.advance_command()
            # Check if root_command is still not None after advancing
            if self.root_command is not None:
                self.root_command.periodic()
            else:
                pass

        # if coms are enabled, send the UDP packet to the server.
        if self.enable_coms:
            send_data_to_server(self.client_socket, self.server_address, stored_messages_txt)

    def set_command_group(self, head: Command):
        self.root_command = head

    def shutdown(self):
        pass

    def add_topics(self, *args):
        """
        add all topics using args

        i.e., add_topics(topic1,topic2,topic3,etc)
        """
        for topic in args:
            self.topics.append(topic)

    def add_subscribers(self, *args):
        """
        add all subscribers using args

        i.e., add_subscribers(sub1,sub2,sub3,sub4)
        """
        for sub in args:
            self.subscribers.append(sub)

    def begin_log(self):
        """
        if we aren't simulating and logging is enabled, then begin a new log file.
        """
        if not self.is_sim and self.should_log:
            self.writing_file_name = "log_" + str(int(time.time())) + ".csv"
            with open(self.writing_file_name, 'w') as self.f:
                self.f.write("Time, TopicMessageDictionaries\n")

    def init_hardware(self):
        """
        initialize hardware, it's honestly okay to do your own initialization and pass those into pyrose topics.

        Make sure you do your own error handling if that is the case.
        """
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
        """
        Ensures that two topics are not named the exact same thing due to the dictionary like nature of pyROSe.
        """
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
