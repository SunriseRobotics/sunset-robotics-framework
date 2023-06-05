class TopicCircularDependency(Exception):
    def __init__(self, *args: object) -> None:
        super().__init__(*args)

class TopicNameCollision(Exception):
    def __init__(self, *args: object) -> None:
        super().__init__(*args)

class SubscriberNameCollision(Exception):
    def __init__(self, *args: object) -> None:
        super().__init__(*args)
