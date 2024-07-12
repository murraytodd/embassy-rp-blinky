# Hello world Python TCP and UDP Servers

The following script will listen to TCP port 9932 for connections and will
then return all received messages with the prefix "hello, ".

```python
import socket

HOST = '0.0.0.0'  # Listen on all interfaces
PORT = 9932

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen()
    while True:
        conn, addr = s.accept()
        print(f'Connected by {addr}')
        with conn:
            while True:
                # Receive data from client (maximum 1024 bytes)
                data = conn.recv(1024).decode(errors="ignore")
                if not data: 
                    print("Connection Ended")
                    break
                print(f'Received {data.strip()}')
                # Respond with "hello, " + message
                response = f"hello, {data}"
                conn.sendall(response.encode())
                print(f'Response {response.strip()} sent')
```

The following script will do the same thing, but listening to
the *UDP* port 9932, sending the response to the sender's specified 
return address.

```python
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
```