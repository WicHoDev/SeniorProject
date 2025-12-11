import os

def writeFile(file, data):
    f = open(file, "a")
    f.write(str(data[0]) + ' ' + str(data[1])+'\n')
    data = []

def readFile(file, data):
    print("Reading to File")
    if os.path.exists(file):
        print("File not create")
        return
    f = open(file, "r")
    return f.readline(data)


def sendingData(data):
    print("Sending Data")

