import numpy as np
import json 
import re

def split_outside_brackets(s, delimiter=','):
    # This regex pattern will split the string correctly:
    pattern = r',\s*(?![^{}]*})'
    return re.split(pattern, s)

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


def parse_line(line: str):
    """
    This function takes in a line in the format 'time,json' and returns a tuple containing the time as a float and the JSON data as a dictionary.
    """
    if line:
        time, message = line.strip().split(',', 1)
        time = float(time)
        return time, message
    else:
        return None
    


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
        message_as_dict[key] = split_outside_brackets(message_as_dict[key])
        message_as_dict[key][0] = json.loads(message_as_dict[key][0])
        message_as_dict[key][1] = float(message_as_dict[key][1])
        message_as_dict[key][2] = float(message_as_dict[key][2])

    return message_as_dict




if __name__ == "__main__":
    tricky_string = '{"imu_orientation_z_deg": 0, "encoder_2": 0, "encoder_1": 0, "imu_orientation_x_deg": 0, "imu_orientation_y_deg": 0}, 1687274057.6506414, 0.003597736358642578'
    print(split_outside_brackets(tricky_string))
    print(len(split_outside_brackets(tricky_string)))