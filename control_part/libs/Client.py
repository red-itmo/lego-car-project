import pickle
import socket

""" Class for socket on ev3 """


class Client:
    def __init__(self, port):
        self.s = socket.socket()
        try:
            self.s.connect(('DESKTOP-S1K69E9', port))
            print("Connection established")
        except socket.error:
            print( "Closing socket in init" )
            self.s.close()
            raise RuntimeError("Can't establish the connection to the server with given port number")

    def __del__(self):
        print( "Closing socket... But why?" )
        self.s.close()

    def get_data(self):
        # print("Im waiting for some data...")
        data = self.s.recv(4096)
        if not data:
            return False
        else:
            decoded_data = pickle.loads(data)
            self.s.send(pickle.dumps(str(1)))
            return decoded_data
