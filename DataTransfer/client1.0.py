import socket

#create client
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#local Host
client.connect(('129.113.132.31', 4444))

receive = open("dataReceived.txt", "w")

while True:
    data = client.recv(1024).decode()

    print(data)
    receive.write(data)

    if data == "quit":
        client.close()
