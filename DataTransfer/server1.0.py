import socket

server = socket.socket()
server.bind(('0.0.0.0', 4444))
server.listen(1)
print("lisening...")

conn, addr = server.accept()
print(f"connection from: *Ip Addres*")
data = conn.recv(1024).decode()
print("Received: ", data)
conn.send("Hello Client".encode())
conn.close()
