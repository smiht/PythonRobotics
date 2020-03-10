import  csv
import socket
import struct
import pickle
UDP_IP = "localhost"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, # Internet
             socket.SOCK_DGRAM) # UDP


#csv_file = open('/home/iisri/matlabCode/simulinkObstacleAvoidance/my_iOS_device 200219 12_56_34.csv')
csv_file = open('/home/iisri/matlabCode/git_repo_new/simulinkObstacleAvoidance/my_iOS_device 200219 12_56_34.csv')

leader_data = csv.reader(csv_file)
next(leader_data,None)
#print(leader_data)
i = 0
for row in leader_data:

    MESSAGE = pickle.dumps([float(row[4]),float(row[5]),float(row[7]),float(row[16])])
    print(i)
    i = i+1
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
