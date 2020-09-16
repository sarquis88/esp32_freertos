import socket
import sys

# Create a UDP socket
sock = socket.socket( socket.AF_INET, socket.SOCK_STREAM )

sock.bind( ( "192.168.100.3", 20000 ) )
packet = bytearray( 1 )
sock.listen(1)

while True:
    connection, client_address = sock.accept()
    try:
        while True:
            data = connection.recv( 1 )
            print('received {!r}'.format(data))
    finally:
        connection.close()


## Receive
#while 1:
#    sock.recvfrom_into( packet )
#    print( int.from_bytes( packet, byteorder='big', signed=False ) 
# ) 

