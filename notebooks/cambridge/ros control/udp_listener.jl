using Sockets

# Receiver settings
HOST = "127.0.0.1"  
PORT = 12345      # UDP port to listen on

# Create UDP socket
sock = UDPSocket()
bind(sock, HOST, PORT)

println("Listening on $HOST:$PORT...")

# Receive UDP packet
while true
    data, addr = receivefrom(sock)
    println("Received message: $(String(data)) from $addr")
end
