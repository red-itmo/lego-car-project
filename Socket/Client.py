import pickle
import socket

""" Class for socket on ev3 """


class Client:
    def __init__(self, port):
        self.s = socket.socket()
        try:
            self.s.connect(('RamiUbuntu', port))
            print("Connection established")
        except socket.error:
            self.s.close()
            raise RuntimeError("Can't establish the connection to the server with given port number")

    def __del__(self):
        self.s.close()

    def get_data(self):
        self.s.send( str.encode(str(1)) )
        self.s.settimeout(1)
        data = self.s.recv(2048)
        data = pickle.loads(data)
        if not data:
            return False
        else:
            self.s.send(str.encode(str(1)))
            return data


