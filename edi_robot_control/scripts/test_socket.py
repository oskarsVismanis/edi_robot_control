import socket
adress = '10.13.136.118'
port = 5000
server_address = (adress, port)
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(server_address)