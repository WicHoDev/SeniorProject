import socket

#create client
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#local Host
client.connect(('129.113.132.31', 4444))

client.send("Hello From Client".encode())
print(client.recv(1024).decode())
