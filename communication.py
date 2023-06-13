import socket
import logging

G_LAND_REQUEST = "LANDR"
G_READY = "READY"
G_TAKEOFF = "TAKEOFF"
G_FAIL = "FAIL"

class Server:
    LAND_REQUEST = G_LAND_REQUEST
    READY = G_READY
    TAKEOFF = G_TAKEOFF
    FAIL = G_FAIL
    
    def __init__(self, host, port) -> None:
        try:
            self.__s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.__s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.__s.bind((host, port))
            self.__s.listen()

        except Exception as e:
            logging.error("Error initializing the server: " + str(e), exc_info=True)

    #None timeout = Infinitum time, loop = 1, runs only once
    def wait_conn(self, timeout = None, loop = 1):
        """
        timeout = seconds e.g., 0.5.
        When timeout=None, waits for message indefinitely
        
        loop = numbers of attemps with timeout

        combining timeout with loop might be interesting when the system has several threads runnning
        """
        self.__s.settimeout(timeout)
        while loop > 0:
            loop = loop - 1
            try:
                conn, addr = self.__s.accept()
                self.__curr_conn = conn
                self.__curr_addr = addr

            except Exception as e:
                if not isinstance(e, TimeoutError):
                    logging.error("Error connection " + str(e), exc_info=True)
                    return False
        return True

    def receive_msg(self, type=None, timeout=None):
        """
        type = LAND_REQUEST, READY, TAKEOFF, FAIL, None.
        When type=None, returns the actual data received.
        When type=e.g., TAKEOFF, returns True if data is a TAKEOFF message.

        timeout = seconds e.g., 0.5.
        When timeout=None, waits for message indefinitely
        """
        self.__curr_conn.settimeout(timeout)
        try:
            data = self.__curr_conn.recv(1024).decode()
            if type is None: 
                return data
            else:
                return type in data

        except Exception as e:
            if not isinstance(e, TimeoutError):
                logging.error("Error getting message " + str(e), exc_info=True)
            return False

    def send_msg(self, msg = READY):
        try:
            self.__curr_conn.sendall(msg.encode())
            return True
        
        except Exception as e:
            logging.error("Error sending message " + str(e), exc_info=True)
            return False

    def close_curr(self):
        self.__curr_conn.close()
        self.__curr_conn = None
        self.__curr_addr = None

    def close(self):
        self.__s.close()


class Client:
    LAND_REQUEST = G_LAND_REQUEST
    READY = G_READY
    TAKEOFF = G_TAKEOFF
    FAIL = G_FAIL

    def __init__(self, host:str, port:int) -> None:
        self.__host = host
        self.__port = port

    def conn(self):
        try:
            self.__s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.__s.connect((self.__host, self.__port))
            return True
        
        except Exception as e:
            logging.error("Error connecting " + str(e), exc_info=True)
            return False

    def send_msg(self, msg = LAND_REQUEST):
        try:
            self.__s.sendall(msg.encode())
            return True
        
        except Exception as e:
            logging.error("Error sending message " + str(e), exc_info=True)
            return False       

    def receive_msg(self, type=None, timeout=None):
        """
        type = LAND_REQUEST, READY, TAKEOFF, FAIL, None.
        When type=None, returns the actual data received.
        When type=e.g., TAKEOFF, returns True if data is a TAKEOFF message.

        timeout = seconds e.g., 0.5.
        When timeout=None, waits for message indefinitely
        """
        self.__s.settimeout(timeout)
        try:
            data = self.__s.recv(1024).decode()

            if type is None: 
                return data
            else:
                return type in data
            
        except Exception as e:
            if not isinstance(e, TimeoutError):
                logging.error("Error getting message " + str(e), exc_info=True)
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
        if(c.conn()):
            print(str(os.getpid()) + str(c.send_msg(c.LAND_REQUEST)))
            while not c.receive_msg(c.READY, timeout=0.1) : pass
            print(str(os.getpid()) + str(True))
            time.sleep(0.1)
            print(str(os.getpid()) + str(c.send_msg(c.READY)))
            print(str(os.getpid()) + str(c.receive_msg(c.TAKEOFF)))
            print(str(os.getpid()) + str(c.send_msg(c.READY)))
            c.close()
            print(str(os.getpid()) + "done")

    if "s" in running:
        s = Server("0.0.0.0", 2810) #beginning
        for i in range(20):
            print(f"Round{i}")
            s.wait_conn() #wait for a connection
            print(s.receive_msg(s.LAND_REQUEST)) #get msg of land request
            time.sleep(2)
            print(s.send_msg(s.READY)) #send ready only when the drone is allowed to land
            print(s.receive_msg(s.READY)) #receive a ready only when the drone has landed
            time.sleep(0.5) #change battery
            print(s.send_msg(s.TAKEOFF)) #send takeoff to drone
            print(s.receive_msg(s.READY)) #receive ready after drone being flying
            s.close_curr()
        s.close()