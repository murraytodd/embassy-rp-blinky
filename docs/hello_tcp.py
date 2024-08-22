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