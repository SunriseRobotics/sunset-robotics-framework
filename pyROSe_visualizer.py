import socket
import threading
from architecture.topicLogUtil import *
from visualization.triad import *
import time
import select
import cProfile

TYPES_3D_PLOT = {
    "TRANSLATION3D": ["X", "Y", "Z"],
    "ANGLE_RAD": ["X", "Y", "Z"],
    "ORIENTATION_RAD": ["X", "Y", "Z"],
}

server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 4096 * 10)  # 10 times the previous buffer size

# Bind the socket to a specific address and port
server_address = ('192.168.1.120', 12345)  # Change to your needs
server_socket.bind(server_address)
triads = {}

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(0.3, 0.7)
ax.set_ylim(0.3, 0.7)
ax.set_zlim(0.3, 0.7)
triads_lock = threading.Lock()
triads["ORIGIN"] = TriadVector(ax,origin=[0,0,0],length=0.1)


def update_triads(data):
    with triads_lock:
        for key in data.keys():
            for type_ in TYPES_3D_PLOT.keys():
                if type_ in key:
                    # remove the end from the string
                    triad_key = key.replace("_" + type_, "")
                    if triad_key not in triads.keys():
                        triads[triad_key] = TriadVector(ax, origin=[0.5, 0.5, 0.5], length=0.1)
                    split_data = split_outside_brackets(data[key])
                    vector_data = json.loads(split_data[0])

                    if "TRANSLATION3D" in type_:
                        triads[triad_key].set_position([vector_data["X"], vector_data["Y"], vector_data["Z"]])
                    elif "ANGLE_RAD" in type_ or "ORIENTATION_RAD" in type_:
                        triads[triad_key].set_rotation([vector_data["X"], vector_data["Y"], vector_data["Z"]])


def update(_):
    with triads_lock:
        artists = []
        for key, triad in triads.items():
            if triad.rotation_data is not None:  # Only try to use rotation_data if it's not None
                triad.set_rotation(triad.rotation_data)
                artists.extend(triad.get_artists())

        ax.figure.canvas.draw()
        ax.figure.canvas.flush_events()
        return artists


def main():
    start = time.time()
    while True:
        try:
            # This will raise a socket.error exception if there's nothing to read
            data, address = server_socket.recvfrom(4096, socket.MSG_DONTWAIT)
            # Convert the data from a byte string to a string
            data = data.decode()

            # Convert the string to a JSON object
            _, data = parse_line(data)

            data = json.loads(data)
            print(data)
            update_triads(data)
        except socket.error:
            pass


if __name__ == "__main__":
    server_thread = threading.Thread(target=main)
    server_thread.start()

    ani = FuncAnimation(fig, update, blit=True, interval=50, repeat=False, cache_frame_data=True)
    plt.show()
    server_socket.close()
