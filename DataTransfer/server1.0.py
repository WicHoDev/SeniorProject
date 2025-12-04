import socket

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(('0.0.0.0', 4444))
server.listen(1)

data = open("data.txt", "r")

client, addr = server.accept()
while client:
    i = data.read()
    client.send(i.encode())

    i = input("Want to exit? ")
    if i == "quit":
        client.send(i.encode())
        client.close()
