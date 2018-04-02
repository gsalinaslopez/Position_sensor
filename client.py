import socket

HOST = '127.0.0.1'    # The remote host
PORT = 6666             # The same port as used by the server
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    x = input()
    y = x + '\n'

    #s.sendall(b'start\n')
    s.sendall(y.encode('utf-8'))
    while True:
        data = s.recv(1024)
        print(repr(data))
    s.close()
