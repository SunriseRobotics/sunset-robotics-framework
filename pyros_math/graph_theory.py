from architecture.architecture_relationships import Topic, Subscriber

def dependecy_sort(topics: list) -> list:
    '''
    When iterating through the topics to publish messages, we should publish messages without dependencies first, then publish messages with dependencies.
    '''
    result = []
    visited = set()

    def dfs(topic):
        if topic in visited:
            return
        visited.add(topic)
        for subscriber in topic.subscribers:
            if isinstance(subscriber,Topic):
                dfs(subscriber)
        result.append(topic)

    for topic in topics:
        if topic not in visited:
            dfs(topic)
    
    return result[::-1]

def find_connected_subgraphs(topics: list) -> list:
    '''
    Find all the connected subgraphs in the graph of topics and subscribers.
    '''
    visited = set()
    subgraphs = []

    def dfs(topic):
        if topic in visited:
            return
        visited.add(topic)
        subgraph.append(topic)
        for subscriber in topic.subscribers:
            if isinstance(subscriber,Topic):
                dfs(subscriber)

    for topic in topics:
        if topic not in visited:
            subgraph = []
            dfs(topic)
            subgraphs.append(subgraph)
    
    return subgraphs

def is_cycle_present(connected_topics: list) -> list:
    '''
    determines if the topics have a cycle, indicating a circular dependency.
    '''
    tortise = connected_topics[0]
    hare = connected_topics[0]

    while True:

        if len(tortise.subscribers) == 0:
            # if the tortise has gotten to the end with no remaining subscribers, then there is no cycle
            return False
        tortise = tortise.subscribers[0]

        # hare moves twice as fast as tortise
        if len(hare.subscribers) == 0:
            # if the hare has gotten to the end with no remaining subscribers, then there is no cycle
            return False
        
        hare = hare.subscribers[0]
        if len(hare.subscribers) == 0:
            # if the hare has gotten to the end with no remaining subscribers, then there is no cycle
            return False
        
        hare = hare.subscribers[0]

        # if hare or tortise is not a subclass of Topic, then there is no cycle
        if not isinstance(hare,Topic) or not isinstance(tortise,Topic):
            return False


        if hare is None or len(hare.subscribers) == 0:
            # if there is no hare, or if the hare has gotten to the end with no remaining subscribers, then there is no cycle
            return False
        if hare == tortise:
            # if the hare and tortise are the same, then there is a cycle
            return True


def cycle_is_present_in_any(all_topics: list) -> bool:
    subgraphs = find_connected_subgraphs(all_topics)
    for subgraph in subgraphs:
        if is_cycle_present(subgraph):
            return True
    return False
        