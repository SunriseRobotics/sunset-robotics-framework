import numpy as np
import json 


def dump_file_contents(file_name: str):
    """
    This function takes in a file name and returns a tuple of lists. The first list is the list of all the times in the file, and the second list is the list of all the messages in the file. The messages are stored as a list of dictionaries.
    """
    times = []
    messages = []
    with open(file_name, 'r') as f:
        for line in f:
            line = line.strip()
            if line:
                time, message = line.split(',', 1)
                if time == "Time":
                    continue
                times.append(float(time))
                messages.append(message)

    return times, messages



def construct_dictionary_of_messages_vs_time(time_stamps, messages):
    """
    This function takes in a list of times and a list of messages and returns a dictionary of messages vs time. The messages are stored as a list of dictionaries.
    """
    dictionary_of_messages_vs_time = {}
    for time, message in zip(time_stamps, messages):
        dictionary_of_messages_vs_time[time] = message
    return dictionary_of_messages_vs_time


def get_message_at_time(time, dictionary_of_messages_vs_time):
    """
    This function takes in a time and a dictionary of messages vs time and returns the message at the given time.
    """
    message_as_dict = json.loads(dictionary_of_messages_vs_time[time])
    for key in message_as_dict:
        message_as_dict[key] = message_as_dict[key].split(',')
        message_as_dict[key][0] = json.loads(message_as_dict[key][0])
        message_as_dict[key][1] = float(message_as_dict[key][1])
        message_as_dict[key][2] = float(message_as_dict[key][2])

    return message_as_dict
    


