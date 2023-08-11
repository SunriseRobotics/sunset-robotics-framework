from architecture.architecture_relationships import Topic, Subscriber


def dependency_sort(topics: list) -> list:
    """
    When iterating through the topics to publish messages, we should publish messages without dependencies first, then publish messages with dependencies.
    """
    result = []
    visited = set()

    def dfs(topic):
        if topic in visited:
            return
        visited.add(topic)
        for subscriber in topic.subscribers:
            if isinstance(subscriber, Topic):
                dfs(subscriber)
        result.append(topic)

    for topic in topics:
        if topic not in visited:
            dfs(topic)

    return result[::-1]


def find_connected_subgraphs(topics: list) -> list:
    """
    Find all the connected subgraphs in the graph of topics and subscribers.
    """
    visited = set()
    subgraphs = []

    def dfs(topic):
        if topic in visited:
            return
        visited.add(topic)
        subgraph.append(topic)
        for subscriber in topic.subscribers:
            if isinstance(subscriber, Topic):
                dfs(subscriber)

    for topic in topics:
        if topic not in visited:
            subgraph = []
            dfs(topic)
            subgraphs.append(subgraph)

    return subgraphs


def is_cycle_present(connected_topics: list) -> bool:
    """
    determines if the topics have a cycle, indicating a circular dependency.
    """
    tortoise = connected_topics[0]
    hare = connected_topics[0]

    while True:
        if not isinstance(hare, Topic):
            return False

        if len(tortoise.subscribers) == 0:
            # if the tortoise has gotten to the end with no remaining subscribers, then there is no cycle
            return False
        tortoise = tortoise.subscribers[0]

        # hare moves twice as fast as tortoise
        if len(hare.subscribers) == 0:
            # if the hare has gotten to the end with no remaining subscribers, then there is no cycle
            return False

        hare = hare.subscribers[0]
        if len(hare.subscribers) == 0:
            # if the hare has gotten to the end with no remaining subscribers, then there is no cycle
            return False

        hare = hare.subscribers[0]

        # if hare or tortoise is not a subclass of Topic, then there is no cycle
        if not isinstance(hare, Topic) or not isinstance(tortoise, Topic):
            return False

        if hare is None or len(hare.subscribers) == 0:
            # if there is no hare, or if the hare has gotten to the end with no remaining subscribers, then there is
            # no cycle
            return False
        if hare == tortoise:
            # if the hare and tortoise are the same, then there is a cycle
            return True


def cycle_is_present_in_any(all_topics: list) -> bool:
    subgraphs = find_connected_subgraphs(all_topics)
    for subgraph in subgraphs:
        if is_cycle_present(subgraph):
            return True
    return False


def generate_network_graph(topics, subscribers):
    import networkx as nx
    import matplotlib.pyplot as plt
    G = nx.DiGraph()

    # Iterate through the list of topics
    for topic in topics:
        G.add_node(topic.name, color='blue')  # Topics are represented as blue nodes
        # Add an edge from this topic to each of its subscribers
        for subscriber in topic.subscribers:
            G.add_edge(topic.name, subscriber.name)

    # Iterate through the list of subscribers (which are not topics)
    for subscriber in subscribers:
        if subscriber not in topics:
            G.add_node(subscriber.name, color='red')  # Subscribers are represented as red nodes

    # Draw the graph
    pos = nx.spring_layout(G)  # positions for all nodes
    labels_pos = {node: (pos[node][0], pos[node][1] + 0.1) for node in G.nodes()}  # positions for all labels
    colors = [node[1]['color'] for node in G.nodes(data=True)]
    nx.draw(G, pos, node_color=colors, with_labels=False, arrows=True)
    nx.draw_networkx_labels(G, labels_pos)
    plt.show()
