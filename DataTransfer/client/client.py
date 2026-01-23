import socket
import numpy as np
import matplotlib.pyplot as plt

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect(("129.113.132.31", 4444))

buffer = ""

f = []
s11_r = []
s11_i = []

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
        f.append(int(freq))
        real = float(parts[1])
        s11_r.append(real)
        imag = float(parts[2])
        s11_i.append(imag)

        print(f"Frequency: {freq} Hz")
        print(f"S11 Real: {real}")
        print(f"S11 Imag: {imag}")
        print("-----------------------------")
client.close()

print('\n')
print(len(f))
print(len(s11_r))
print(len(s11_i))

mag = []

f = np.array(f)
s11_r = np.array(s11_r)
s11_i = np.array(s11_i)

mag = np.sqrt(s11_r**2 + s11_i**2)
mag_dB = 10*np.log10(mag)

plt.plot(f, mag_dB)
plt.xlabel("Frequency")
plt.ylabel("mag (dB)")
plt.title("S11 Magnitude")
plt.savefig("plot.png")
print("Saved plot.png")
