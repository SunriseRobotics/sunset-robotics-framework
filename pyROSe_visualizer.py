import socket
import threading
import os
from architecture.topicLogUtil import *
from visualization.VisualizerGeometry import *
from network_constants import *
import paramiko

TYPES_3D_PLOT = {
    "TRANSLATION3D": ["X", "Y", "Z"],
    "ANGLE_RAD": ["X", "Y", "Z"],
    "ORIENTATION_RAD": ["X", "Y", "Z"],
    "POSE2D": ["X", "Y", "THETA"],
    "SO3": ["ROLL", "PITCH", "YAW"],
    "SE3": ["X", "Y", "Z", "ROLL", "PITCH", "YAW"]
}

server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 4096 * 10)  # 10 times the previous buffer size

# Bind the socket to a specific address and port


server_address = (visualizer_ip, 12345)  # Change to your needs
server_socket.bind(server_address)
triads = {}
rectangular_prisms = {}

plt.style.use("ggplot")
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
boundL, boundH = -1, 2
ax.set_xlim(boundL, boundH)
ax.set_ylim(boundL, boundH)
ax.set_zlim(boundL, boundH)
triads_lock = threading.Lock()
triads["ORIGIN"] = TriadVector(ax, origin=[0, 0, 0], length=0.1)
rectangular_prisms["test"] = RectangularPrism(ax, origin=[0.1, 0.1, 0.1], dimensions=(1, 1, 1))
removed_item_names = []


def update_triads(data):
    with triads_lock:
        for key in data.keys():
            for type_ in TYPES_3D_PLOT.keys():
                if type_ in key:
                    # remove the end from the string
                    triad_key = key.replace("_" + type_, "")

                    if triad_key in removed_item_names:
                        continue

                    if triad_key not in triads.keys():
                        triads[triad_key] = TriadVector(ax, origin=[0.5, 0.5, 0.5], length=0.1)
                    split_data = split_outside_brackets(data[key])
                    vector_data = json.loads(split_data[0])

                    if "TRANSLATION3D" in type_:
                        triads[triad_key].set_position([vector_data["X"], vector_data["Y"], vector_data["Z"]])
                    elif "ANGLE_RAD" in type_ or "ORIENTATION_RAD" in type_:
                        triads[triad_key].set_rotation([vector_data["X"], vector_data["Y"], vector_data["Z"]])
                    elif "POSE2D" in type_:
                        triads[triad_key].set_position([vector_data["X"], vector_data["Y"], 0])
                        triads[triad_key].set_rotation([0, 0, vector_data["THETA"]])
                    elif "SO3" in type_:
                        triads[triad_key].set_rotation([vector_data["PITCH"], vector_data["ROLL"], vector_data["YAW"]])
                    elif "SE3" in type_:
                        triads[triad_key].set_rotation([vector_data["PITCH"], vector_data["ROLL"], vector_data["YAW"]])
                        triads[triad_key].set_position([vector_data["X"], vector_data["Y"], vector_data["Z"]])


def user_commands():
    """
    command syntax

    add - adds an element to the 3d visualization
    examples:
    - add triad triad_name 30, 30, 30, 0, 0, 0
    - remove triad triad_name
    - remove prism prism_name
    """

    while True:
        user_input = input("pyROSe - viz: ")
        user_input = user_input.split(" ")
        command = user_input[0]
        command = command.lower()
        print(user_input)
        if command == "add":
            type_to_add = user_input[1].lower()
            if type_to_add == "triad":
                name = user_input[2]
                if name in triads.keys():
                    print("{} already exists!".format(name))
                    continue
                try:
                    x, y, z = float(user_input[3]), float(user_input[4]), float(user_input[5])
                except IndexError:
                    print("something went wrong with parsing, try syntax again")
                    continue
                roll, pitch, yaw = 0, 0, 0

                if len(user_input) >= 9:
                    # there exists angular elements
                    roll = np.degrees(float(user_input[6]))
                    pitch = np.degrees(float(user_input[7]))
                    yaw = np.degrees(float(user_input[8]))

                triads[name] = TriadVector(ax, origin=[x, y, z], length=0.3)
                triads[name].set_rotation([roll, pitch, yaw])
        if command == "remove":
            print("removing")
            type_to_remove = user_input[1].lower()
            print(type_to_remove)
            name_to_remove = user_input[2]
            print(name_to_remove)
            if type_to_remove == "triad":
                try:
                    del triads[name_to_remove]
                    removed_item_names.append(name_to_remove)
                except KeyError:
                    print("Name was not found in the list")
        if command == "ls" or command == "list":
            print("Triads:")
            for triad in triads.keys():
                name = triad
                position = triads[name].origin
                orientation = triads[name].orientation
                x = position[0]
                y = position[1]
                z = position[2]
                roll = orientation[0]
                pitch = orientation[1]
                yaw = orientation[2]
                print("Name: {} ; Position: (X:{} Y:{} Z:{}) "
                      "Orientation (deg): (Roll:{} Pitch:{} Yaw:{})"
                      .format(triad, x, y, z, np.degrees(roll), np.degrees(pitch), np.degrees(yaw)))
        if command == "run":
            file = user_input[1]
            run_script_on_rpi()
            print('ssh ' + robot_hostname + '@' + robot_ip + ' "python3 ./Documents/pyROS/' + file + '"')


def update(_):
    with triads_lock:
        artists = []
        for key, triad in triads.items():
            if triad.rotation_data is not None:  # Only try to use rotation_data if it's not None
                triad.set_rotation(triad.rotation_data)
                artists.extend(triad.get_artists())

        for key, prism in rectangular_prisms.items():
            if prism.rotation_data is not None:
                prism.set_rotation(prism.rotation_data)
                artists.extend(prism.get_artists())

        ax.figure.canvas.draw()
        ax.figure.canvas.flush_events()
        return artists


def main():
    while True:
        try:
            # This will raise a socket.error exception if there's nothing to read
            data, address = server_socket.recvfrom(4096, socket.MSG_DONTWAIT)
            # Convert the data from a byte string to a string
            data = data.decode()
            # Convert the string to a JSON object
            _, data = parse_line(data)
            data = json.loads(data)
            update_triads(data)
        except socket.error:
            pass


def run_script_on_rpi(hostname, port, username, password, script_name):
    # Create a new SSH client
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    # Connect to the Raspberry Pi
    client.connect(hostname, port, username, password)

    # The command to run
    command = f'python3 /Documents/pyROS/{script_name}'

    # Execute the command
    stdin, stdout, stderr = client.exec_command(command)

    # Print any output from the command
    for line in stdout:
        print(line.strip('\n'))

    # Close the connection
    client.close()


if __name__ == "__main__":
    server_thread = threading.Thread(target=main)
    user_command_thread = threading.Thread(target=user_commands)

    server_thread.start()
    user_command_thread.start()

    ani = FuncAnimation(fig, update, blit=True, interval=50, repeat=False, cache_frame_data=True)
    plt.show()
    server_socket.close()
