from architecture.scheduler import Scheduler
from architecture.architecture_relationships import Command, DelayCommand, Topic, Subscriber, Message
import numpy as np
from sunset_math.geometry import SE3, SO3
from sunset_math.TrapezoidProfile import *
from sunset_math.AutomaticControl.LinearControl import PID, SimpleFeedforward, calculateDerivativePositionControl

from sunset_math.graph_theory import generate_network_graph
import math


def main():
    is_running_on_robot = False


    scheduler = Scheduler(False, "log_file_if_reading_log.csv", enable_coms=True)

    # topics
    # subscribers
    # link topics to subscribers
    # topic.add_subscriber(subscriber)


    command = DelayCommand(0.3,scheduler.sysTimeTopic);
    # use .add_next to add chains of commands.  This example command does nothing for 0.3 seconds.
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
            # if broken, then use this area to stop hardware. This is a good place to put a breakpoint.
            return


if __name__ == "__main__":
    main()
