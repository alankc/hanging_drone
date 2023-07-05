import socket
import logging
import os
import time

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
        """
        Bind and Listen server
        """
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
                if not (isinstance(e, TimeoutError) or isinstance(e, ConnectionResetError)):
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
            if not (isinstance(e, TimeoutError) or isinstance(e, ConnectionResetError)):
                logging.error("Error getting message " + str(e), exc_info=True)
                return None
            return False

    def send_msg(self, msg:str=LAND_REQUEST, value:str=""):
        """
        Send a text msg to the currently active connection
        
        Return true if delivered with success
        """
        try:
            m = msg + value 
            self.__curr_conn.sendall(m.encode())
            return True
        
        except Exception as e:
            logging.error("Error sending message " + str(e), exc_info=True)
            return False

    def close_curr(self):
        """
        Close current active connection
        """
        self.__curr_conn.close()
        self.__curr_conn = None
        self.__curr_addr = None

    def close(self):
        """
        Close server
        """
        self.__s.close()


class Client:
    LAND_REQUEST = G_LAND_REQUEST
    READY = G_READY
    TAKEOFF = G_TAKEOFF
    FAIL = G_FAIL

    def __init__(self, host:str, port:int) -> None:
        self.__host = host
        self.__port = port

    def conn(self, timeout = 1):
        """
        Try to connect to the server at the host and port provided in the constructor

        Return true if connected
        """
        try:
            self.__s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.__s.settimeout(timeout)
            self.__s.connect((self.__host, self.__port))
            return True
        
        except Exception as e:
            if not (isinstance(e, ConnectionRefusedError) or isinstance(e, TimeoutError) or isinstance(e, ConnectionResetError)):
                logging.error("Error connecting " + str(e), exc_info=True)
            return False

    def send_msg(self, msg:str=LAND_REQUEST, value:str=""):
        """
        Send a text msg to the currently active connection
        
        Return true if delivered with success
        """
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
            if not (isinstance(e, TimeoutError) or isinstance(e, ConnectionResetError)):
                logging.error("Error connecting " + str(e), exc_info=True)
                return None
            return False
    
    def close(self):
        """
        Close connection to the server
        """
        self.__s.close()

#adapted from https://stackoverflow.com/questions/54479347/simplest-way-to-connect-wifi-python
class WiFiFinder:
    def __init__(self, interface:str="wlxd8ec5e0a30b5"):
        self.__interface = interface

    def check_and_connect(self, ssid:str, password:str):
        """
        Check if is possible to connect to the SSID and connect to it

        Return true if connected
        """
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
                return self.__connect(ssid, password)
            except Exception as e:
                logging.error(f"Error connecting to {ssid}: " + str(e), exc_info=True)

        return False

    def __connect(self, ssid, password):
        """
        Connect to the ssid using the password

        Return true if connected
        """
        cmd = f"nmcli d wifi connect {ssid} password {password}"
        try:
            if os.system(cmd) != 0: # This will run the command and check connection
                raise Exception()
        except:
            raise # Not Connected
        else:
            return True # Connected

class D2RS:
    def __init__(self, host:str, port:int, interface="wlxd8ec5e0a30b5", ssid:str="TELLO-98FD38", password="TELLOBISG") -> None:
        self.__host = host
        self.__port = port
        self.__ssid = ssid
        self.__password = password
        self.__wifi = WiFiFinder(interface)

    def land_request(self, timeout=0.5):
        """
        Send a land request to the battery recharge station

        Return true if permission to land granted
        """
        c = Client(self.__host, self.__port)
        check = False
        if c.conn(timeout): #If connection worked
            if c.send_msg(Client.LAND_REQUEST, self.__ssid): #if the message was sent
                if c.receive_msg(msg=Client.READY, timeout=timeout): #If received message ready ()
                        check = c.send_msg(Client.READY) #return true if sent the message ready
            c.close()
        return check #return false for any failure

    def takeoff_request(self, timeout=0.5):
        """
        Send a takeoff request to the battery recharge station (BRS). 
        It sends the SSID so the BRS knows the drone that is going to land

        Return the SSID of the drone to connect or None
        """
        c = Client(self.__host, self.__port)
        ssid = None
        if c.conn(timeout): #If connection worked
            if c.send_msg(Client.TAKEOFF): #if the message was sent
                msg = c.receive_msg(timeout=timeout) #If received message TAKEOFF SSID
                if isinstance(msg, str) and Client.TAKEOFF in msg:
                    check = c.send_msg(Client.READY) #return true if sent the message ready
                    if check:
                        ssid = msg.split()[1]
            c.close()
        return ssid #return None for any failure or the ssid name

    def wifi_conect(self, ssid:str):
        """
        Connect to the Wi-Fi SSID informed
        """        
        if ssid is None:
            return self.__wifi.check_and_connect(self.__ssid, self.__password)
        
        if self.__wifi.check_and_connect(ssid, self.__password):
            self.__ssid = ssid
            return True
        
        return False

class RS2D:
    def __init__(self, host:str = "127.0.0.1", port:int = 2810) -> None:
        self.__port = port
        self.__host = host

    def land_request(self, timeout=0.5):
        """
        Respond to a land request

        Return the ssid of the drone requesting land
        """
        ssid = None
        self.__s = Server(self.__host, self.__port)
        
        if self.__s.conn():
            if self.__s.wait_conn(timeout=timeout):
                msg = self.__s.receive_msg(timeout=timeout)
                if isinstance(msg, str) and Server.LAND_REQUEST in msg: #if received a land request
                    if self.__s.send_msg(Server.READY): #if sent the messsage ready
                        if self.__s.receive_msg(msg=Server.READY, timeout=timeout): #if received a message ready
                            ssid = msg.split()[1] #get the ssid
                self.__s.close_curr()
            
            self.__s.close()

        return ssid

    def takeoff_request(self, ssid, timeout=0.5):
        """
        Respond to a takeoff request with the SSID of the drone ready to takeoff

        Return true if a client accepted to takeoff
        """
        check = False
        self.__s = Server(self.__host, self.__port)

        if self.__s.conn():
            if self.__s.wait_conn(timeout=timeout):
                if self.__s.receive_msg(msg=Server.TAKEOFF, timeout=timeout):  #If received message TAKEOFF
                    if self.__s.send_msg(Server.TAKEOFF, ssid): #if sent takeof with the SSID
                        check = self.__s.receive_msg(msg=Server.READY, timeout=timeout) #if received ready to takeoff
                self.__s.close_curr()
            
            self.__s.close()

        return check

if __name__ == "__main__":
    import sys
    import time
    import os

    if len(sys.argv) == 2:
        running = sys.argv[1]

    if "C" in running:
        d2rs = D2RS("127.0.0.1", 2810, "wlxd8ec5e0a30b5", "TELLO-98FD38")
        while True:
            if d2rs.land_request(1):
                print("LAND OK")
            else:
                print("DELAY")
                time.sleep(0.1)

    if "S" in running:
        rs2d = RS2D("127.0.0.1", 2810)
        import random
        while True:
            tr = random.randint(1, 2000) / 100
            print("-------------------------------")
            print(f"Sleep time: {tr}")
            time.sleep(tr)
            t1 = time.time()
            print(rs2d.land_request(1))
            print(f"Time of land request = {time.time() - t1}")

    if "c" in running:
        d2rs = D2RS("127.0.0.1", 2810, "wlxd8ec5e0a30b5", "TELLO-98FD38")

        count = 0
        while not d2rs.land_request():
            print(f"{count} No response in land request!")
            count = count + 1
            time.sleep(1)

        print("Success in land request!")

            
        count = 0
        res_ssid = d2rs.takeoff_request()
        while res_ssid is None:
            print(f"{count} No response in takeoff request!")
            count = count + 1
            res_ssid = d2rs.takeoff_request()

        if res_ssid:
            count = 5
            check = False
            while not check and count > 0:
                print(f"Trying to connect to Wifi: {res_ssid}")
                check = d2rs.wifi_conect(res_ssid)
                count = count - 1

            if count == 0 and not check:
                print("Failed to connect wifi")
            else:
                print("Ready to takeoff")
        else:
            print("No response from takeoff request")

    if "s" in running:
        rs2d = RS2D("127.0.0.1", 2810)
        while True:
            print("Waiting Connection")
            res_ssid = rs2d.land_request(0.1)
            #You have to keep the ssids from the drone that are ready to takeof
            #You must run the takeoff_request only if you have drone available to takeoff 
            if res_ssid:
                break

        while True:
            print("Changing battery")
            if rs2d.takeoff_request(res_ssid, 0.1):
                break