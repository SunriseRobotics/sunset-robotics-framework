import unittest


class Topic:
    def __init__(self):
        self.subscribers = []


def is_cycle_present(connected_topics: list) -> bool:
    def dfs(node, visited):
        if node in visited:
            return True

        if not isinstance(node, Topic) or not node.subscribers:
            return False

        visited.add(node)

        for subscriber in node.subscribers:
            if dfs(subscriber, visited):
                return True

        visited.remove(node)
        return False

    visited = set()
    return dfs(connected_topics[0], visited)


class TestCycleDetection(unittest.TestCase):

    def test_no_cycle(self):
        a = Topic()
        b = Topic()
        c = Topic()
        d = Topic()

        a.subscribers = [b]
        b.subscribers = [c]
        c.subscribers = [d]

        self.assertFalse(is_cycle_present([a]))

    def test_simple_cycle(self):
        a = Topic()
        b = Topic()

        a.subscribers = [b]
        b.subscribers = [a]

        self.assertTrue(is_cycle_present([a]))

    def test_complex_cycle(self):
        a = Topic()
        b = Topic()
        c = Topic()
        d = Topic()

        a.subscribers = [b, c]
        b.subscribers = [c]
        c.subscribers = [d]
        d.subscribers = [b]

        self.assertTrue(is_cycle_present([a]))

    def test_self_loop(self):
        a = Topic()
        a.subscribers = [a]

        self.assertTrue(is_cycle_present([a]))

    def test_disconnected_graph(self):
        a = Topic()
        b = Topic()
        c = Topic()
        d = Topic()

        a.subscribers = [b]
        c.subscribers = [d]

        self.assertFalse(is_cycle_present([a]))


if __name__ == "__main__":
    unittest.main()
