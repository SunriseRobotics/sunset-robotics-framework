from pyrose_math.geometry import *
from pyrose_math.TrapezoidProfile import *
from pyrose_math.graph_theory import *
from architecture.scheduler import Scheduler
from architecture.architecture_relationships import *
import unittest
from unittest.mock import patch
import math


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
        subscriber.store_messages("test_topic", message)
        self.assertIn("test_topic", subscriber.messages)

    def test_subscriber_periodic(self):
        subscriber = TestSubscriber(False)
        with patch.object(subscriber, 'subscriber_periodic', wraps=subscriber.subscriber_periodic) as mocked_method:
            subscriber.periodic()
            mocked_method.assert_called_once()

    def test_subscriber_periodic_sim(self):
        subscriber = TestSubscriber(True)
        with patch.object(subscriber, 'subscriber_periodic_sim',
                          wraps=subscriber.subscriber_periodic_sim) as mocked_method:
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
        self.assertTrue(command.first_run_occurred)

    def test_parallel_command(self):
        command1 = TestCommand([])
        command2 = TestCommand([])
        parallel_command = ParallelCommand([command1, command2])
        parallel_command.first_run()
        self.assertTrue(parallel_command.first_run_occurred)

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
        self.assertTrue(command3.first_run_occurred)

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


class TestTrapezoidProfile(unittest.TestCase):
    def setUp(self):
        self.max_accel = 30
        self.max_decel = 30
        self.max_vel = 50
        self.target_position_positive = 300
        self.target_position_negative = -300
        self.profile_positive = TrapezoidProfile(self.max_accel, self.max_decel, self.max_vel,
                                                 self.target_position_positive)
        self.profile_negative = TrapezoidProfile(self.max_accel, self.max_decel, self.max_vel,
                                                 self.target_position_negative)

    def test_positive_profile_duration(self):
        expected_profile_duration = self.profile_positive.dt1 + self.profile_positive.dt2 + self.profile_positive.dt3
        self.assertEqual(self.profile_positive.profileDuration, expected_profile_duration)

    def test_negative_profile_duration(self):
        expected_profile_duration = self.profile_negative.dt1 + self.profile_negative.dt2 + self.profile_negative.dt3
        self.assertEqual(self.profile_negative.profileDuration, expected_profile_duration)

    def test_end_state_positive(self):
        final_state = self.profile_positive.getState(self.profile_positive.profileDuration)
        self.assertAlmostEqual(final_state.x, self.target_position_positive, 4)
        self.assertAlmostEqual(final_state.v, 0, 4)
        self.assertAlmostEqual(final_state.a, 0, 4)

    def test_end_state_negative(self):
        final_state = self.profile_negative.getState(self.profile_negative.profileDuration)
        self.assertAlmostEqual(final_state.x, self.target_position_negative, 4)
        self.assertAlmostEqual(final_state.v, 0, 4)
        self.assertAlmostEqual(final_state.a, 0, 4)

    def test_mid_state_positive(self):
        mid_state = self.profile_positive.getState(self.profile_positive.profileDuration / 2)
        self.assertTrue(mid_state.v <= self.max_vel)
        self.assertTrue(math.fabs(mid_state.a) <= self.max_accel)

    def test_mid_state_negative(self):
        mid_state = self.profile_negative.getState(self.profile_negative.profileDuration / 2)
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

        sorted_topics = dependency_sort(topics)

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

        topics = [topic1, topic2, topic3, topic4, topic5, topic6, topic7]

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

        topics = [topic1, topic2, topic3, topic7]

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

        topics = [topic1, topic2, topic3, topic7]

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

        topics = [topic1, topic2, topic3, topic7]

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

        topics = [topic1, topic2, topic3, topic4, topic5, topic6]

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

        topics = [topic1, topic2, topic3, topic4, topic5, topic6]

        assert cycle_is_present_in_any(topics) == True


class TestQuaternion(unittest.TestCase):
    def test_mul(self):
        q1 = Quaternion(1, 0, 0, 0)  # Identity quaternion
        q2 = Quaternion.from_angle_axis(np.pi, np.array([0, 0, 1]))  # 180-degree rotation about Z axis
        q3 = q1 * q2
        np.testing.assert_allclose([q3.w, q3.x, q3.y, q3.z], [0, 0, 0, 1], atol=1e-6)

    def test_to_rotation_matrix(self):
        q = Quaternion.from_angle_axis(np.pi / 2, np.array([0, 0, 1]))  # 90-degree rotation about Z axis
        R = q.to_rotation_matrix()
        np.testing.assert_allclose(R, np.array([
            [0, -1, 0],
            [1, 0, 0],
            [0, 0, 1]
        ]), atol=1e-6)


class TestSO3(unittest.TestCase):
    def test_from_euler(self):
        so3 = SO3.from_euler(np.pi / 2, 0, 0)
        expected_rotation_matrix = np.array([
            [1, 0, 0],
            [0, 0, -1],
            [0, 1, 0]
        ])
        np.testing.assert_allclose(so3.rotation_matrix, expected_rotation_matrix, rtol=1e-5, atol=1e-8)

    def test_mul(self):
        so3_1 = SO3.from_euler(np.pi / 2, 0, 0)
        so3_2 = SO3.from_euler(0, np.pi / 2, 0)
        result = so3_1 * so3_2
        expected_rotation_matrix = np.dot(so3_1.rotation_matrix,
                                          so3_2.rotation_matrix)  # Update the expected_rotation_matrix
        np.testing.assert_allclose(result.rotation_matrix, expected_rotation_matrix, rtol=1e-5, atol=1e-8)

    def test_inverse(self):
        so3 = SO3.from_euler(np.pi / 2, 0, 0)
        result = so3.inverse()
        expected_rotation_matrix = np.array([
            [1, 0, 0],
            [0, 0, 1],
            [0, -1, 0]
        ])
        np.testing.assert_allclose(result.rotation_matrix, expected_rotation_matrix, rtol=1e-5, atol=1e-8)


class TestSE3(unittest.TestCase):

    def setUp(self):
        self.tolerance = 1e-6

    def test_transform_to(self):
        # Define transforms
        origin = SE3(SO3.from_euler(0, 0, 0), np.array([0, 0, 0]))
        A = SE3.from_euler_and_translation(np.pi / 4, np.pi / 4, np.pi / 4, 1, 2, 3)
        B = SE3.from_euler_and_translation(np.pi / 2, np.pi / 2, np.pi / 2, 4, 5, 6)

        # Test transform_to
        transform_A_to_B = A.transform_to(B)
        expected_transform = B * A.inverse()

        self.assertTrue(
            np.allclose(transform_A_to_B.rotation.rotation_matrix, expected_transform.rotation.rotation_matrix,
                        rtol=self.tolerance, atol=self.tolerance))
        self.assertTrue(np.allclose(transform_A_to_B.translation, expected_transform.translation, rtol=self.tolerance,
                                    atol=self.tolerance))

    def test_relative_to(self):
        # Define transforms
        origin = SE3(SO3.from_euler(0, 0, 0), np.array([0, 0, 0]))
        A = SE3.from_euler_and_translation(np.pi / 4, np.pi / 4, np.pi / 4, 1, 2, 3)
        B = SE3.from_euler_and_translation(np.pi / 2, np.pi / 2, np.pi / 2, 4, 5, 6)
    
        # Test relative_to
        A_relative_to_B = A.relative_to(B)
        expected_relative = B.inverse() * A
        self.assertTrue(
            np.allclose(A_relative_to_B.rotation.rotation_matrix, expected_relative.rotation.rotation_matrix,
                        rtol=self.tolerance, atol=self.tolerance))
        self.assertTrue(np.allclose(A_relative_to_B.translation, expected_relative.translation, rtol=self.tolerance,
                                    atol=self.tolerance))

    def test_from_euler_and_translation(self):
        se3 = SE3.from_euler_and_translation(np.pi / 2, 0, 0, 1, 2, 3)
        expected_rotation = SO3.from_euler(np.pi / 2, 0, 0)
        expected_translation = np.array([1, 2, 3])
        np.testing.assert_array_almost_equal(se3.rotation.rotation_matrix, expected_rotation.rotation_matrix)
        np.testing.assert_array_almost_equal(se3.translation, expected_translation)

    def test_mul(self):
        rotation_matrix1 = np.array([[0, 0, 1],
                                     [0, 1, 0],
                                     [-1, 0, 0]])
        rotation_matrix2 = np.array([[0, -1, 0],
                                     [1, 0, 0],
                                     [0, 0, 1]])

        translation1 = np.array([1, 2, 3])
        translation2 = np.array([4, 5, 6])

        se3_1 = SE3(SO3(rotation_matrix1), translation1)
        se3_2 = SE3(SO3(rotation_matrix2), translation2)

        result = se3_1 * se3_2

        expected_rotation_matrix = np.dot(rotation_matrix1, rotation_matrix2)  # rotation matrices are multiplied
        expected_rotation = SO3(expected_rotation_matrix)

        expected_translation = np.dot(rotation_matrix1,
                                      translation2) + translation1  # translations are added after rotation

        np.testing.assert_array_almost_equal(result.rotation.rotation_matrix, expected_rotation.rotation_matrix)
        np.testing.assert_array_almost_equal(result.translation, expected_translation)

    def test_inverse(self):
        rotation_matrix = np.array([[0, 0, 1],
                                    [0, 1, 0],
                                    [-1, 0, 0]])

        translation = np.array([1, 2, 3])

        se3 = SE3(SO3(rotation_matrix), translation)

        result = se3.inverse()

        expected_rotation_matrix = rotation_matrix.transpose()  # the inverse of a rotation matrix is its transpose
        expected_rotation = SO3(expected_rotation_matrix)
        expected_translation = - np.dot(expected_rotation_matrix,
                                        se3.translation)  # note the updated expected translation

        np.testing.assert_array_almost_equal(result.rotation.rotation_matrix, expected_rotation.rotation_matrix)
        np.testing.assert_array_almost_equal(result.translation, expected_translation)


if __name__ == '__main__':
    unittest.main()
