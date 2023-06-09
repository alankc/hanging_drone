import socket

__READY = 0
__FAIL = 1

class RechargeStationComm:

    def __init__(self, host, port) -> None:
        self.__s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.__s.bind((host, port))
            self.__s.listen()
        except Exception as e:
            print("Error initializing the server: " + e)
            self.__s.close()

    #None timeout = Infinitum time, loop = 1, runs only once
    def get_message(self, timeout = None, loop = 1):
        self.__s.settimeout(timeout)
        while loop > 0:
            loop = loop - 1
            try:
                conn, addr = self.__s.accept()
                self.__curr_conn = conn
                self.__curr_addr = addr
                while True:
                    data = conn.recv(1024)
                    if not data:
                        break

            except Exception as e:
                if not isinstance(e, TimeoutError):
                    conn.close()
                    self.__curr_conn = None
                    self.__curr_addr = None
                    print("Error getting message" + e)

    def send_response(self, response = __READY):
        try:
            self.__curr_conn.sendall(__READY)
        except Exception as e:
            print("Error getting message" + e)
        finally:
            self.__curr_conn.close()    


class DroneComm:
    def __init__(self) -> None:
        pass