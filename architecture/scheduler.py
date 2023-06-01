
from architecture.architecture_relationships import Command


class Scheduler:
    def __init__(self):
        self.topics = []
        self.subscribers = []
        self.is_sim = False
        self.throw_exception_on_init_failure = True
        self.root_command = None

    def initialize(self):
        visited_subs = []
        for sub in self.subscribers:
            for sub_other in visited_subs:
                if sub.name == sub_other.name:
                    sub.name = sub.name + "0"
            
            success = sub.initialize_hardware()
            if self.throw_exception_on_init_failure and not success:
                raise RuntimeError(f"Hardware for Subscriber, '{sub.name}' failed to initialize, aborting init")
            sub.is_sim = self.is_sim

    def advance_command(self):
        if self.root_command and self.root_command.is_complete():
            self.root_command = self.root_command.next_command

    def periodic(self):

        if not self.root_command is None and not self.root_command.first_run_occured:
            self.root_command.first_run()
        
        self.advance_command()

        # Check if root_command is still not None after advancing
        if self.root_command is not None:
            self.root_command.periodic()
        else:
            print("No further command to execute")

        for topic in self.topics:
            topic.publish_periodic()

        for sub in self.subscribers:
            sub.periodic()
    def set_command_group(self, head: Command):
        self.root_command = head


