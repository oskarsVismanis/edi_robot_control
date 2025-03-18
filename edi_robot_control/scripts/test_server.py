
import socket
import pickle

adress = '0.0.0.0'
port = 5000
server_adress = (adress, port)
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(server_adress)
server_socket.listen(1)

print("Connection testing...")
client_socket, client_adress = server_socket.accept()
print(f"Connection succesful with {client_adress}")

recv_data = client_socket.recv(4096)
obj_coords = pickle.loads(recv_data)

print("Closing connection...")
client_socket.close()
server_socket.close()

print(obj_coords)
# print(obj_coords[0])
# print(obj_coords[1])