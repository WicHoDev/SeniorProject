import socket
import pyvisa #read from USB

#create client
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#connect to server
# client.connect(('129.113.131.75', 4444))  #school IP
client.connect(('192.168.1.64', 4444))      #house IP


receive = open("dataReceived.txt", "w")
while True:
    data = client.recv(1024).decode()

    print(data)
    receive.write(data)

    if data == "quit":
        client.close()


