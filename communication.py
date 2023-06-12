import socket

LAND_REQUEST = "LANDR"
READY = "READY"
TAKEOFF = "TAKEOFF"
FAIL = "FAIL"

class Server:
    def __init__(self, host, port) -> None:
        try:
            self.__s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.__s.bind((host, port))
            self.__s.listen()
        except Exception as e:
            print("Error initializing the server: " + e)
            self.__s.close()

    #None timeout = Infinitum time, loop = 1, runs only once
    def wait_conn(self, timeout = None, loop = 1):
        self.__s.settimeout(timeout)
        while loop > 0:
            loop = loop - 1
            try:
                conn, addr = self.__s.accept()
                self.__curr_conn = conn
                self.__curr_addr = addr
            except Exception as e:
                if not isinstance(e, TimeoutError):
                    conn.close()
                    self.__curr_conn = None
                    self.__curr_addr = None
                    print("Error getting message" + e)

    def receive_msg(self, type = None):
        try:
            data = ""
            while True:
                d = self.__curr_conn.recv(1024)
                if not d: break
                data = data + str(d)

            if not type: 
                return data
            else:
                return type in data

        except Exception as e:
            self.__curr_conn.close()
            self.__curr_conn = None
            self.__curr_addr = None
            print("Error getting message" + e)

    def send_msg(self, msg = READY):
        try:
            self.__curr_conn.sendall(msg)
        except Exception as e:
            print("Error sending message" + e)
        finally:
            self.__curr_conn.close()

    def close_curr(self):
        self.__curr_conn.close()
        self.__curr_conn = None
        self.__curr_addr = None

    def close(self):
        self.__s.close()


class Client:
    def __init__(self, host:str, port:int) -> None:
        self.__host = host
        self.__port = port

    def conn(self):
        try:
            self.__s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.__s.connect(self.__host, self.__port)
        except Exception as e:
            print("Error sending message" + e)
            self.__s.close()     

    def send_msg(self, msg = LAND_REQUEST):
        try:
            self.__s.sendall(msg)
        except Exception as e:
            print("Error sending message" + e)
            self.__s.close()              

    def receive_msg(self, type = None):
        try:
            data = ""
            while True:
                d = self.__s.recv(1024)
                if not d: break
                data = data + str(d)

            if not type: 
                return data
            else:
                return type in data
            
        except Exception as e:
            self.__s.close()
            self.__s = None
            print("Error getting message" + e)
    
    def close(self):
        self.__s.close()

    def is_type(self, data, type = [LAND_REQUEST, READY, TAKEOFF, FAIL]):
        return type in data

if __name__ == "__main__":
    c = Client("", 2810)
    c.is_type("asdadsa", LAND_REQUEST)