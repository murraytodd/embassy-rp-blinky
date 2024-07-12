import socket

HOST = '0.0.0.0'  # Listen on all interfaces
PORT = 9932

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the socket to the port
sock.bind((HOST, PORT))
print(f"Server listening for messages on port {PORT}")

# Listen for incoming messages
while True:
    data, address = sock.recvfrom(1024)
    message = data.decode()
    
    # Print the message and address
    print(f"Received message {message} from {address}")
    reply = f"hello, {message}"
    # Echo a response to the sender
    sock.sendto(reply.encode(), address)
    print(f"Sent response '{reply}' back in response.")