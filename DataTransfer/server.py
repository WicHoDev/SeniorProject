import socket

#create socket
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(('0.0.0.0', 4444))

server.listen(1)

while True:
    client, addr = server.accept()
    print(client.recv(1024).decode())
    client.send("Hello From Server".encode())
