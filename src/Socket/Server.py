# import socket
# import pickle
# """ Class for socket on PC """
#
#
# class Server:
#     def __init__(self, port):
#         self.s = socket.socket()
#         self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
#         print(socket.gethostname())
#         self.client_info = ()
#         try:
#             self.s.bind(('', port))
#         except OSError:
#             self.s.close()
#             raise RuntimeError("Can't bind the port")
#         self.s.listen(1)
#
#     """ Destructor (maybe?) """
#
#     def __del__(self):
#         if self.client_info:
#             self.client_info[0].close()
#         self.s.close()
#
#     """ Return tuple: ( client, address ), wait for connection from client """
#     """ If no client connected within 60 sec, close all sockets and return false (I hope) """
#     # TODO: add timeout or something else (so it will terminate on its own if no clients connect)
#     # Test timeout
#     def ready(self):
#         try:
#             self.s.settimeout(60)
#             self.client_info = self.s.accept()
#             return self.client_info
#         except socket.error:
#             self.s.close()
#             return False
#
#     """ def send returns True if data is sent and acknowledgment received, else false  """
#     # TODO: how to cast data to string(or byte)
#     # Dict to string
#     def send(self, data):
#         data = pickle.dumps(data)
#         if self.client_info:
#             self.client_info[0].send(data)
#             # if no acknowledgment within 0.1 sec - whole thing gonna crash
#             try:
#                 self.s.settimeout(1)
#                 data_is_sent = self.client_info[0].recv(1024)
#                 if data_is_sent == str.encode("1"):
#                     return True
#                 else:
#                     return False
#
#             except socket.error:
#                 self.client_info[0].close()
#                 self.s.close()
#                 return False
#         else:
#             return False
import socket
import pickle
""" Class for socket on PC """


class Server:
    def __init__(self, port):
        self.s = socket.socket()
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        print(socket.gethostname())
        self.client_info = ()
        try:
            self.s.bind(('', port))
        except OSError:
            self.s.close()
            raise RuntimeError("Can't bind the port")
        self.s.listen(1)

    """ Destructor (maybe?) """

    def __del__(self):
        if self.client_info:
            self.client_info[0].close()
        self.s.close()

    """ Return tuple: ( client, address ), wait for connection from client """
    """ If no client connected within 60 sec, close all sockets and return false (I hope) """
    # TODO: add timeout or something else (so it will terminate on its own if no clients connect)
    # Test timeout
    def ready(self):
        try:
            self.s.settimeout(60)
            self.client_info = self.s.accept()
            return self.client_info
        except socket.error:
            self.s.close()
            return False

    """ def send returns True if data is sent and acknowledgment received, else false  """
    # TODO: how to cast data to string(or byte)
    # Dict to string
    def send(self, data):
        data = pickle.dumps(data)
        if self.client_info:
            #print(self.client_info[0])
            self.client_info[0].send(data)
            # if no acknowledgment within 0.1 sec - whole thing gonna crash
            self.s.settimeout(1)
            data_is_sent = self.client_info[0].recv(1024)
            if data_is_sent:
                return True
            else:
                return False
        else:
            return False
