import socket

def start_server():
    # Create a UDP socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Bind the socket to a specific address and port
    server_address = ('192.168.1.120', 12345)  # Change to your needs
    server_socket.bind(server_address)

    while True:
        # print("\nWaiting to receive message...")
        data, address = server_socket.recvfrom(4096)

        # print(f"Received {len(data)} bytes from {address}")
        print(data)

if __name__ == "__main__":
    start_server()