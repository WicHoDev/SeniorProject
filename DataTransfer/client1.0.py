import socket

client = socket.socket()
client.connect(('localhost', 4444))
client.send("Hello Server".encode())
responde = client.recv(1024).decode()
print("from Server: ", responde)
client.close()
