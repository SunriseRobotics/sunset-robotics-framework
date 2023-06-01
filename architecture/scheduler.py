import unittest
from unittest.mock import MagicMock, patch
from architecture_relationships import *
import time
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
        if self.root_command is None:
            raise RuntimeError("No command set, cannot run periodic tasks")

        if not self.root_command.first_run_occured:
            self.root_command.first_run()
        
        self.advance_command()
        self.root_command.periodic()

        for topic in self.topics:
            topic.publish_periodic()

        for sub in self.subscribers:
            sub.periodic()

    def set_command_group(self, head: Command):
        self.root_command = head


class TestCommand(Command):
    def first_run_behavior(self):
        pass

    def periodic(self):
        pass

    def is_complete(self):
        return True

class TestSubscriber(Subscriber):
    def subscriber_periodic(self):
        pass

class TestTopic(Topic):
    def generate_messages_periodic(self):
        return {}
    def subscriber_periodic(self):
        return super().subscriber_periodic()


    

class ArchitectureRelationshipsTest(unittest.TestCase):
    
    def setUp(self):
        self.scheduler = Scheduler()

    def test_scheduler_initialization(self):
        try:
            self.scheduler.initialize()
        except Exception as e:
            self.fail("Scheduler initialization failed with exception: {}".format(str(e)))

    def test_scheduler_periodic_without_command(self):
        with self.assertRaises(RuntimeError):
            self.scheduler.periodic()

    def test_scheduler_set_command_group(self):
        command = TestCommand([])
        self.scheduler.set_command_group(command)
        self.assertIsInstance(self.scheduler.root_command, Command)

    def test_scheduler_advance_command(self):
        command1 = TestCommand([])
        command2 = TestCommand([])
        command1.setNext(command2)
        self.scheduler.set_command_group(command1)
        self.scheduler.advance_command()
        self.assertEqual(self.scheduler.root_command, command2)

    def test_subscriber_store_messages(self):
        subscriber = TestSubscriber(False)
        message = Message({"test": "test"})
        subscriber.store_messages(message, "test_topic")
        self.assertIn("test_topic", subscriber.messages)

    def test_subscriber_periodic(self):
        subscriber = TestSubscriber(False)
        with patch.object(subscriber, 'subscriber_periodic', wraps=subscriber.subscriber_periodic) as mocked_method:
            subscriber.periodic()
            mocked_method.assert_called_once()

    def test_subscriber_periodic_sim(self):
        subscriber = TestSubscriber(True)
        with patch.object(subscriber, 'subscriber_periodic_sim', wraps=subscriber.subscriber_periodic_sim) as mocked_method:
            subscriber.periodic()
            mocked_method.assert_called_once()

    def test_command_set_next(self):
        command1 = TestCommand([])
        command2 = TestCommand([])
        command1.setNext(command2)
        self.assertEqual(command1.next_command, command2)

    def test_command_first_run(self):
        command = TestCommand([])
        command.first_run()
        self.assertTrue(command.first_run_occured)

    def test_parallel_command(self):
        command1 = TestCommand([])
        command2 = TestCommand([])
        parallel_command = ParallelCommand([command1, command2])
        parallel_command.first_run()
        self.assertTrue(parallel_command.first_run_occured)

    def test_topic_add_subscriber(self):
        topic = TestTopic()
        subscriber = TestSubscriber(False)
        topic.add_subscriber(subscriber)
        self.assertIn(subscriber, topic.subscribers)

    def test_topic_publish_periodic(self):
        topic = TestTopic()
        subscriber = TestSubscriber(False)
        topic.add_subscriber(subscriber)
        with patch.object(subscriber, 'store_messages', wraps=subscriber.store_messages) as mocked_method:
            topic.publish_periodic()
            mocked_method.assert_called_once()

if __name__ == "__main__":
    unittest.main()
