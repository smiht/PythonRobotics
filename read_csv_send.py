import  csv
import socket
import struct
import pickle
UDP_IP = "localhost"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, # Internet
             socket.SOCK_DGRAM) # UDP


#csv_file = open('/home/iisri/matlabCode/simulinkObstacleAvoidance/my_iOS_device 200219 12_56_34.csv')
csv_file = open('/home/jd/matlab_code/simulinkObstacleAvoidance/my_iOS_device 200219 12_56_34.csv')

leader_data = csv.reader(csv_file)
next(leader_data,None)
#print(leader_data)
for row in leader_data:
    print(row[4],row[5],row[7],row[16])
    MESSAGE = pickle.dumps([row[4],row[5],row[7],row[16]])
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
