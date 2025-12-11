import socket

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect(("129.113.132.31", 4444))

buffer = ""

while True:
    data = client.recv(1024)
    if not data:
        break

    buffer += data.decode()

    # Process complete lines
    while "\n" in buffer:
        line, buffer = buffer.split("\n", 1)
        parts = line.split(",")

        if len(parts) != 3:
            continue

        freq = float(parts[0])
        real = float(parts[1])
        imag = float(parts[2])

        print(f"Frequency: {freq} Hz")
        print(f"S11 Real: {real}")
        print(f"S11 Imag: {imag}")
        print("-----------------------------")

client.close()
