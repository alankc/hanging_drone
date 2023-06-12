import socket

LAND_REQUEST = "LANDR"
READY = "READY"
TAKEOFF = "TAKEOFF"
FAIL = "FAIL"

class Server:
    def __init__(self, host, port) -> None:
        try:
            self.__s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.__s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.__s.bind((host, port))
            self.__s.listen()

        except Exception as e:
            print("Error initializing the server: " + str(e))

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
                    print("Error connection " + str(e))
                    return False
        return True

    def receive_msg(self, type = None):
        try:
            data = self.__curr_conn.recv(1024).decode()
            if type is None: 
                return data
            else:
                return type in data

        except Exception as e:
            print("Error getting message " + str(e))
            return False

    def send_msg(self, msg = READY):
        try:
            self.__curr_conn.sendall(msg.encode())
            return True
        
        except Exception as e:
            print("Error sending message " + str(e))
            return False

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
            self.__s.connect((self.__host, self.__port))
            return True
        
        except Exception as e:
            print("Error connecting " + str(e))
            return False

    def send_msg(self, msg = LAND_REQUEST):
        try:
            self.__s.sendall(msg.encode())
            return True
        
        except Exception as e:
            print("Error sending message " + str(e))
            return False       

    def receive_msg(self, type = None):
        try:
            data = self.__s.recv(1024).decode()

            if type is None: 
                return data
            else:
                return type in data
            
        except Exception as e:
            print("Error getting message " + str(e))
            return False
    
    def close(self):
        self.__s.close()

if __name__ == "__main__":
    import sys
    import time
    import os

    if len(sys.argv) == 2:
        running = sys.argv[1]

    if "c" in running:
        time.sleep(2)
        c = Client("", 2810)
        c.conn()
        print(str(os.getpid()) + str(c.send_msg(LAND_REQUEST)))
        print(str(os.getpid()) + str(c.receive_msg(READY)))
        time.sleep(0.1)
        print(str(os.getpid()) + str(c.send_msg(READY)))
        print(str(os.getpid()) + str(c.receive_msg(TAKEOFF)))
        print(str(os.getpid()) + str(c.send_msg(READY)))
        c.close()
        print(str(os.getpid()) + "done")

    if "s" in running:
        s = Server("0.0.0.0", 2810) #beginning
        for i in range(20):
            print(f"Round{i}")
            s.wait_conn() #wait for a connection
            print(s.receive_msg(LAND_REQUEST)) #get msg of land request
            print(s.send_msg(READY)) #send ready only when the drone is allowed to land
            print(s.receive_msg(READY)) #receive a ready only when the drone has landed
            time.sleep(0.5) #change battery
            print(s.send_msg(TAKEOFF)) #send takeoff to drone
            print(s.receive_msg(READY)) #receive ready after drone being flying
        s.close()