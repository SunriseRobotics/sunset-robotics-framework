import json
import socket
import threading
from matplotlib.animation import FuncAnimation
from matplotlib import pyplot as plt
import numpy as np

from time import sleep
from architecture.topicLogUtil import *
from visualization.triad import *

TYPES_3D_PLOT = {
    "POSE3D": ["X", "Y", "Z"],
    "ANGLE_RAD": ["X", "Y", "Z"],
    "ORIENTATION_RAD": ["X", "Y", "Z"],
}

server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

# Bind the socket to a specific address and port
server_address = ('192.168.1.120', 12345)  # Change to your needs
server_socket.bind(server_address)

triads = {}

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


def update_triads(data):
    for key in data.keys():
        for type in TYPES_3D_PLOT.keys():
            if type in key:
                if key not in triads.keys():
                    triads[key] = TriadVector(ax, origin=[0,0,0], length=1)  # I've set a fixed origin and length. Please adjust as per your needs.
                split_data = split_outside_brackets(data[key])
                print(split_data[0])
                vector_data = json.loads(split_data[0])

                if "POSE3D" in type:
                    triads[key].set_position([vector_data["X"], vector_data["Y"], vector_data["Z"]])
                elif "ANGLE_RAD" in type or "ORIENTATION_RAD" in type:
                    triads[key].set_rotation([vector_data["X"], vector_data["Y"], vector_data["Z"]])


def update(_):
    artists = []
    for key, triad in triads.items():
        artists.extend(triad.get_artists())
    return artists


def main():
    try:
        print("\nWaiting to receive message...")
        while True:
            data, address = server_socket.recvfrom(4096)

            # convert the data from a byte string to a string
            data = data.decode()

            # convert the string to a json object
            time, data = parse_line(data)

            data = json.loads(data)
            update_triads(data)

            print(data)
            # delay for 0.1 seconds
            sleep(0.1)

            
    except KeyboardInterrupt as e:
        print(e)


if __name__ == "__main__":
    server_thread = threading.Thread(target=main)
    server_thread.start()

    ani = FuncAnimation(fig, update, blit=False, interval=100, repeat=True, cache_frame_data=False)
    plt.show()
    server_socket.close()