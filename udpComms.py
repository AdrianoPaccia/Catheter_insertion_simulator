import socket

 
localIP     = "127.0.0.1"
localPort   = 20001
bufferSize  = 1024

# Create a datagram socket
UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

# Bind to address and ip
UDPServerSocket.bind((localIP, localPort))
UDPServerSocket.settimeout(0.01)

print("UDP server up and listening")


def receiveUDPStats():
    # Listen for incoming datagrams
    try:
        bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
        message = bytesAddressPair[0]
        address = bytesAddressPair[1]
        clientMsg = "Message from Client:{}".format(message)
        clientIP  = "Client IP Address:{}".format(address)
        #print(clientMsg)
        #print(clientIP)
        
        stringdata = message.decode('utf-8')
        messageSplit = stringdata.split(',')
        informationsArray = []
        messageCheck = 0
        #print(stringdata, messageSplit)
        for i in range(len(messageSplit)):
            elm = messageSplit[i]
            if(elm[0] == 'X'):
                messageCheck += 1
                informationsArray.append(float(elm[1:]))
            elif(elm[0] == 'Y'):
                messageCheck += 1
                informationsArray.append(float(elm[1:]))
            elif(elm[0] == 'Z'):
                messageCheck += 1
                informationsArray.append(float(elm[1:]))
            elif(elm[0] == 'Q'):
                messageCheck += 1
                informationsArray.append(float(elm[1:]))
            elif(elm[0] == 'W'):
                messageCheck += 1
                informationsArray.append(float(elm[1:]))
            elif(elm[0] == 'E'):
                messageCheck += 1
                informationsArray.append(float(elm[1:]))
            elif(elm[0] == 'B'):
                messageCheck += 1
                informationsArray.append(float(elm[1:]))
                #print("end of message reached!")
                break
        
        if(messageCheck !=  7):
            #print("message is probably broken, discarding it")
            return None,None
        return informationsArray,address

    except socket.timeout:
        return None,None

def sendUDPForce(force,address):
    # Sending a reply to client
    sendString = f'X{force[0]},Y{force[1]},Z{force[2]}'
    bytesToSend = str.encode(sendString)
    UDPServerSocket.sendto(bytesToSend, address)