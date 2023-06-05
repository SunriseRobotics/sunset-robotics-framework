from pyros_math.kinematics import * 
from pyros_math.geometry import *
from pyros_math.TrapezoidProfile import *
from pyros_math.graph_theory import *
from architecture.scheduler import *
from architecture.architecture_relationships import *
import unittest
from unittest.mock import patch

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
        subscriber.store_messages("test_topic",message)
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

    def test_parallel_command_with_next_in(self):
        command1 = TestCommand([])
        command2 = TestCommand([])
        command3 = TestCommand([])
        command1.setNext(command3)
        parallel_command = ParallelCommand([command1, command2])
        # two iterations of periodic should be enough to run all commands
        parallel_command.periodic()
        parallel_command.periodic()
        # assert that command3 was run
        self.assertTrue(command3.first_run_occured)
        
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

class TestTwist2DMethods(unittest.TestCase):
    def test_twist_to_transform_straight_line(self):
        twist = twist2D(velocity(5), velocity(0), 0)
        time = seconds(1)
        transform = twist.twist_to_transform(time)
        self.assertAlmostEqual(transform.x.get(), 5)
        self.assertAlmostEqual(transform.y.get(), 0)
        self.assertAlmostEqual(transform.theta, 0)

    def test_twist_to_transform_circular_path(self):
        twist = twist2D(vx=velocity(0), vy=velocity(5), wz=math.pi/2)
        transform = twist.twist_to_transform(seconds(1))

        # The expected position would be (10/pi, 10/pi)
        self.assertAlmostEqual(transform.x.get(), 3.183098861837907)
        self.assertAlmostEqual(transform.y.get(), 3.183098861837907)

    def test_twist_to_transform_zero_time(self):
        twist = twist2D(velocity(5), velocity(0), math.pi/2)
        time = seconds(0)
        transform = twist.twist_to_transform(time)
        self.assertAlmostEqual(transform.x.get(), 0)
        self.assertAlmostEqual(transform.y.get(), 0)
        self.assertAlmostEqual(transform.theta, 0)


class TestStateMethods(unittest.TestCase):

    def test_seconds(self):
        self.assertEqual(seconds(5).seconds, 5)

    def test_milliseconds(self):
        self.assertAlmostEqual(milliseconds(5000).seconds, 5)

    def test_microseconds(self):
        self.assertAlmostEqual(microseconds(5000000).seconds, 5)

    def test_tSeconds_add(self):
        time1 = seconds(5)
        time2 = seconds(7)
        self.assertEqual((time1 + time2).seconds, 12)

    def test_tSeconds_sub(self):
        time1 = seconds(5)
        time2 = seconds(7)
        self.assertEqual((time2 - time1).seconds, 2)

    def test_velocity_mul(self):
        v = velocity(3)
        self.assertEqual((v * 2).get(), 6)

    def test_velocity_add(self):
        v1 = velocity(3)
        v2 = velocity(5)
        self.assertEqual((v1 + v2).get(), 8)

    def test_velocity_sub(self):
        v1 = velocity(3)
        v2 = velocity(5)
        self.assertEqual((v2 - v1).get(), 2)

    def test_position_mul(self):
        p = position(3)
        self.assertEqual((p * 2).get(), 6)

    def test_position_add(self):
        p1 = position(3)
        p2 = position(5)
        self.assertEqual((p1 + p2).get(), 8)

    def test_position_sub(self):
        p1 = position(3)
        p2 = position(5)
        self.assertEqual((p2 - p1).get(), 2)

    def test_vec2_add(self):
        v1 = vec2(position(1), position(2))
        v2 = vec2(position(2), position(3))
        v1.add(v2)
        self.assertEqual(v1.x.get(), 3)
        self.assertEqual(v1.y.get(), 5)
    
    def test_vec2_sub(self):
        v1 = vec2(position(1), position(2))
        v2 = vec2(position(2), position(3))
        v1.sub(v2)
        self.assertEqual(v1.x.get(), -1)
        self.assertEqual(v1.y.get(), -1)
    

class TestTrapezoidProfile(unittest.TestCase):
    def setUp(self):
        self.max_accel = 30
        self.max_decel = 30
        self.max_vel = 50
        self.target_position_positive = 300
        self.target_position_negative = -300
        self.profile_positive = TrapezoidProfile(self.max_accel, self.max_decel, self.max_vel, self.target_position_positive)
        self.profile_negative = TrapezoidProfile(self.max_accel, self.max_decel, self.max_vel, self.target_position_negative)

    def test_positive_profile_duration(self):
        expected_profile_duration = self.profile_positive.dt1 + self.profile_positive.dt2 + self.profile_positive.dt3
        self.assertEqual(self.profile_positive.profileDuration, expected_profile_duration)

    def test_negative_profile_duration(self):
        expected_profile_duration = self.profile_negative.dt1 + self.profile_negative.dt2 + self.profile_negative.dt3
        self.assertEqual(self.profile_negative.profileDuration, expected_profile_duration)

    def test_end_state_positive(self):
        final_state = self.profile_positive.getState(self.profile_positive.profileDuration)
        self.assertAlmostEqual(final_state.x, self.target_position_positive,4)
        self.assertAlmostEqual(final_state.v, 0, 4)
        self.assertAlmostEqual(final_state.a, 0, 4)

    def test_end_state_negative(self):
        final_state = self.profile_negative.getState(self.profile_negative.profileDuration)
        self.assertAlmostEqual(final_state.x, self.target_position_negative,4)
        self.assertAlmostEqual(final_state.v, 0 ,4)
        self.assertAlmostEqual(final_state.a, 0, 4)

    def test_mid_state_positive(self):
        mid_state = self.profile_positive.getState(self.profile_positive.profileDuration/2)
        self.assertTrue(mid_state.v <= self.max_vel)
        self.assertTrue(math.fabs(mid_state.a) <= self.max_accel)

    def test_mid_state_negative(self):
        mid_state = self.profile_negative.getState(self.profile_negative.profileDuration/2)
        self.assertTrue(mid_state.v <= -self.max_vel)
        self.assertTrue(math.fabs(mid_state.a) <= self.max_accel)
class DummyTopic(Topic):
    def generate_messages_periodic(self):
        pass  # Add any implementation here if necessary
    def __str__(self) -> str:
        return self.name
    def __repr__(self) -> str:
        return self.name

class AssessTopicSorting(unittest.TestCase):

    def test_sort_topics_by_dependency(self):
        topic1 = DummyTopic('Topic1')
        topic2 = DummyTopic('Topic2')
        topic3 = DummyTopic('Topic3')
        topic4 = DummyTopic('Topic4')

        topic2.add_subscriber(topic1)  # Topic2 is dependent on Topic1
        topic3.add_subscriber(topic2)  # Topic3 is dependent on Topic2
        topic4.add_subscriber(topic1)  # Topic4 is dependent on Topic1

        topics = [topic1, topic2, topic3, topic4]

        sorted_topics = dependecy_sort(topics)

        print("Sorted topics:", [topic.name for topic in sorted_topics])  # Debug print

        self.assertEqual(sorted_topics, [topic4, topic3, topic2, topic1])


class AssessGraphModule(unittest.TestCase):
    def test_find_connected_subgraphs(self):
        topic1 = DummyTopic("Topic1")
        topic2 = DummyTopic("Topic2")
        topic3 = DummyTopic("Topic3")
        topic4 = DummyTopic("Topic4")
        topic5 = DummyTopic("Topic5")
        topic6 = DummyTopic("Topic6")
        topic7 = DummyTopic("Topic7")


        # we have two connected subgraphs

        topic1.add_subscriber(topic2)
        topic1.add_subscriber(topic3)
        topic1.add_subscriber(topic7)

        topic4.add_subscriber(topic5)
        topic4.add_subscriber(topic6)

        predicted_group_a = [topic1, topic2, topic3, topic7]
        predicted_group_b = [topic4, topic5, topic6]


        topics = [topic1,topic2,topic3,topic4,topic5,topic6,topic7]

        subgraphs = find_connected_subgraphs(topics)

        # there should be two subgraphs
        assert len(subgraphs) == 2
        # the subgraphs should be the same as the predicted subgraphs
        assert subgraphs[0] == predicted_group_a
        assert subgraphs[1] == predicted_group_b

    def test_cycle_detection_no_cycle_1(self):
        topic1 = DummyTopic("Topic1")
        topic2 = DummyTopic("Topic2")
        topic3 = DummyTopic("Topic3")
        topic7 = DummyTopic("Topic7")

        topic1.add_subscriber(topic2)
        topic1.add_subscriber(topic3)
        topic1.add_subscriber(topic7)

        topics = [topic1,topic2,topic3,topic7]

        is_cycle = is_cycle_present(topics)

        assert is_cycle == False

    def test_cycle_detection_no_cycle_2(self):
        topic1 = DummyTopic("Topic1")
        topic2 = DummyTopic("Topic2")
        topic3 = DummyTopic("Topic3")
        topic7 = DummyTopic("Topic7")

        topic1.add_subscriber(topic2)
        topic2.add_subscriber(topic3)
        topic3.add_subscriber(topic7)


        topics = [topic1,topic2,topic3,topic7]

        is_cycle = is_cycle_present(topics)

        assert is_cycle == False

    def test_cycle_detection_cycle_1(self):
        topic1 = DummyTopic("Topic1")
        topic2 = DummyTopic("Topic2")
        topic3 = DummyTopic("Topic3")
        topic7 = DummyTopic("Topic7")

        topic1.add_subscriber(topic2)
        topic2.add_subscriber(topic3)
        topic3.add_subscriber(topic7)
        topic7.add_subscriber(topic1)


        topics = [topic1,topic2,topic3,topic7]

        is_cycle = is_cycle_present(topics)

        assert is_cycle == True

    def test_cylce_detection_one_vertex(self):
        topic1 = DummyTopic("Topic1")

        topics = [topic1]

        is_cycle = is_cycle_present(topics)

        assert is_cycle == False

    def test_for_cycle_in_disconnected_graph_no_cycle(self):
        topic1 = DummyTopic("Topic1")
        topic2 = DummyTopic("Topic2")
        topic3 = DummyTopic("Topic3")
        topic4 = DummyTopic("Topic4")
        topic5 = DummyTopic("Topic5")
        topic6 = DummyTopic("Topic6")
        topic7 = DummyTopic("Topic7")


        # we have two connected subgraphs

        topic1.add_subscriber(topic2)
        topic1.add_subscriber(topic3)
        topic1.add_subscriber(topic7)

        topic4.add_subscriber(topic5)
        topic4.add_subscriber(topic6)

        topics = [topic1,topic2,topic3,topic4,topic5,topic6]

        assert cycle_is_present_in_any(topics) == False

    def test_for_cycle_in_disconnected_graph_with_cycle(self):
        topic1 = DummyTopic("Topic1")
        topic2 = DummyTopic("Topic2")
        topic3 = DummyTopic("Topic3")
        topic4 = DummyTopic("Topic4")
        topic5 = DummyTopic("Topic5")
        topic6 = DummyTopic("Topic6")
        topic7 = DummyTopic("Topic7")


        # we have two connected subgraphs

        topic1.add_subscriber(topic2)
        topic1.add_subscriber(topic3)
        topic1.add_subscriber(topic7)

        topic4.add_subscriber(topic5)
        topic4.add_subscriber(topic6)
        topic5.add_subscriber(topic4)

        topics = [topic1,topic2,topic3,topic4,topic5,topic6]

        assert cycle_is_present_in_any(topics) == True


if __name__ == '__main__':
    unittest.main()

