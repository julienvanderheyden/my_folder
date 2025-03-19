using Sockets

# Sender settings
HOST = ip"172.29.130.141"  # IP address of the receiver (localhost in this case)
PORT = 25342        # UDP port to send data to

#When sending and listening on 172.29.130.141, I got this :
# Listening on 172.29.130.141:25342...
# Received message: Hello, Pyhton! (from JULIA) from ('172.29.128.1', 61580)
# while nothing when I was doing it on 127.0.0.1

# Create UDP socket
sock = UDPSocket()

# Send message
message = "Hello, Pyhton! (from JULIA)"
send(sock,HOST, PORT, message)

println("Sent message: $message to $HOST:$PORT")
close(sock)
