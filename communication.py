import socket
import logging

G_LAND_REQUEST = "LANDR "
G_READY = "READY "
G_TAKEOFF = "TAKEOFF "
G_FAIL = "FAIL "

class Server:
    LAND_REQUEST = G_LAND_REQUEST
    READY = G_READY
    TAKEOFF = G_TAKEOFF
    FAIL = G_FAIL
    
    def __init__(self, host, port) -> None:
        self.__host = host
        self.__port = port

    def conn(self):
        try:
            self.__s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.__s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.__s.bind((self.__host, self.__port))
            self.__s.listen()
            return True
        
        except Exception as e:
            logging.error("Error creating socket " + str(e), exc_info=True)
            return False 

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

    def receive_msg(self, msg=None, timeout=None):
        """
        msg = LAND_REQUEST, READY, TAKEOFF, FAIL, None.
        When msg=None, returns the actual data received.
        e.g., when msg=TAKEOFF, returns True if data is a TAKEOFF message, False if other

        timeout = seconds e.g., 0.5.
        When timeout=None, waits for message indefinitely

        returns in exception: 
                None in timeout
                False in others
        """
        self.__curr_conn.settimeout(timeout)
        try:
            data = self.__curr_conn.recv(1024).decode()
            if msg is None: 
                return data
            else:
                return msg in data

        except Exception as e:
            if not isinstance(e, TimeoutError):
                logging.error("Error getting message " + str(e), exc_info=True)
                return None
            return False

    def send_msg(self, msg:str=LAND_REQUEST, value:str=""):
        try:
            m = msg + value 
            self.__curr_conn.sendall(m.encode())
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

    def send_msg(self, msg:str=LAND_REQUEST, value:str=""):
        try:
            m = msg + value 
            self.__s.sendall(m.encode())
            return True
        
        except Exception as e:
            logging.error("Error sending message " + str(e), exc_info=True)
            return False       

    def receive_msg(self, msg=None, timeout=None):
        """
        msg = LAND_REQUEST, READY, TAKEOFF, FAIL, None.
        When msg=None, returns the actual data received.
        e.g., when msg=TAKEOFF, returns True if data is a TAKEOFF message, False if other

        timeout = seconds e.g., 0.5.
        When timeout=None, waits for message indefinitely

        returns in exception: 
                None in timeout
                False in others
        """
        self.__s.settimeout(timeout)
        try:
            data = self.__s.recv(1024).decode()

            if msg is None: 
                return data
            else:
                return msg in data
            
        except Exception as e:
            if not isinstance(e, TimeoutError):
                logging.error("Error getting message " + str(e), exc_info=True)
                return None
            return False
    
    def close(self):
        self.__s.close()

#adapted from https://stackoverflow.com/questions/54479347/simplest-way-to-connect-wifi-python
class WiFiFinder:
    def __init__(self, interface:str="wlxd8ec5e0a30b5"):
        self.__interface = interface

    def check_and_connect(self, ssid:str):
        
        os.system("nmcli radio wifi off")
        os.system("nmcli radio wifi on")
        time.sleep(1)

        command = f"iwlist {self.__interface} scan | grep -ioE \'ssid:\"{ssid}\"\'"
        result = os.popen(command)
        result = list(result)

        if "Device or resource busy" in result:
                return None
        else:
            ssid_list = [item.lstrip('SSID:').strip('"\n') for item in result]

        for ssid in ssid_list:
            try:
                return self.__connect(ssid)
            except Exception as e:
                logging.error(f"Error connecting to {ssid}: " + str(e), exc_info=True)

        return False

    def __connect(self, ssid, password="TELLOBISG"):
        cmd = f"nmcli d wifi connect {ssid} password {password}"
        try:
            if os.system(cmd) != 0: # This will run the command and check connection
                raise Exception()
        except:
            raise # Not Connected
        else:
            return True # Connected

class D2RS:
    def __init__(self, host:str, port:int, interface="wlxd8ec5e0a30b5", ssid:str="TELLO-98FD38") -> None:
        self.__host = host
        self.__port = port
        self.__ssid = ssid
        self.__wifi = WiFiFinder(interface)

    def land_request(self, timeout=0.5):
        c = Client(self.__host, self.__port)
        check = False
        if c.conn(): #If connection worked
            if c.send_msg(Client.LAND_REQUEST, self.__ssid): #if the message was sent
                if c.receive_msg(msg=Client.READY, timeout=timeout): #If received message ready ()
                        check = c.send_msg(Client.READY) #return true if sent the message ready
            c.close()
        return check #return false for any failure

    def takeoff_request(self, timeout=0.5):
        c = Client(self.__host, self.__port)
        ssid = None
        if c.conn(): #If connection worked
            if c.send_msg(Client.TAKEOFF): #if the message was sent
                msg = c.receive_msg(timeout=timeout) #If received message TAKEOFF SSID
                if isinstance(msg, str) and Client.TAKEOFF in msg:
                    check = c.send_msg(Client.READY) #return true if sent the message ready
                    if check:
                        ssid = msg.split()[1]
            c.close()
        return ssid #return None for any failure or the ssid name

    def wifi_conect(self, ssid:str):
        if self.__wifi.check_and_connect(ssid):
            self.__ssid = ssid
            return True
        return False

class RS2D:
    def __init__(self, port:int) -> None:
        self.__port = port

    def start_server(self):
        self.__s = Server("0.0.0.0", self.__port)
        return self.__s.conn()

    def land_request(self, timeout=0.5):
        ssid = None
        if self.__s.wait_conn(timeout=timeout):
            msg = self.__s.receive_msg(timeout=timeout)
            if isinstance(msg, str) and Server.LAND_REQUEST in msg: #if received a land request
                if self.__s.send_msg(Server.READY): #if sent the messsage ready
                    if self.__s.receive_msg(msg=Server.READY, timeout=timeout): #if received a message ready
                        ssid = msg.split()[1] #get the ssid
            self.__s.close_curr()  
        return ssid

    def takeoff_request(self, ssid, timeout=0.5):
        check = False
        if self.__s.wait_conn(timeout=timeout):
            if self.__s.receive_msg(msg=Server.TAKEOFF, timeout=timeout):  #If received message TAKEOFF
                if self.__s.send_msg(Server.TAKEOFF, ssid): #if sent takeof with the SSID
                    check = self.__s.receive_msg(msg=Server.READY, timeout=timeout) #if received ready to takeoff
            self.__s.close_curr()
        return check

    def stop(self):
        self.__s.close()

if __name__ == "__main__":
    import sys
    import time
    import os

    if len(sys.argv) == 2:
        running = sys.argv[1]

    if "c" in running:
        d2rs = D2RS("", 2810, "wlxd8ec5e0a30b5", "TELLO-98FD38")

        tst = d2rs.land_request()
        if tst:
            print("Success in land request!")
        else:
            print("No response in land request!")
            exit(0)

        res_ssid = d2rs.takeoff_request()
        if res_ssid:
            count = 5
            check = False
            while not check and count > 0:
                print(f"Trying to connect to Wifi: {res_ssid}")
                check = d2rs.wifi_conect(res_ssid)
                count = count - 1

            if count == 0:
                print("Failed to connect wifi")
            else:
                print("Ready to takeoff")
        else:
            print("No response from takeoff request")

    if "s" in running:
        rs2d = RS2D(2810)
        if rs2d.start_server():
            while True:
                print("Waiting Connection")
                res_ssid = rs2d.land_request()
                #You have to keep the ssids from the drone that are ready to takeof
                #You must run the takeoff_request only if you have drone available to takeoff 
                if res_ssid:
                    print("Changing battery")
                    rs2d.takeoff_request(res_ssid)
                time.sleep(0.5)

        rs2d.stop()