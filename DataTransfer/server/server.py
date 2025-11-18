import socket

#create socket
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(('0.0.0.0', 4444))
server.listen(5)
client, addr = server.accept()


data = open("data.txt", "r")
while client:
    i = data.read()
    client.send(i.encode())

    i = input("Want to exit? ")
    if i == "quit":
        client.send(i.encode())
        client.close()


