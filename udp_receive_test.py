import socket
import pickle
IP = "localhost"
PORT = 5005
sock = socket.socket(socket.AF_INET,
                   socket.SOCK_DGRAM) # UDP PROTOCOL
sock.bind((IP, PORT))
while True:
  data, addr = sock.recvfrom(1024) # buffer = 1024 bytes
  message = pickle.loads(data)
  print("received message:", message)
