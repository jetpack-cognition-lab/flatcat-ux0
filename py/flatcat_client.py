#!/usr/bin/python

import sys, socket, time

# globals
address  = "flatcat07.local"
port     = 7332
bufsize  = 4096
MSG = '\033[93m' #orange terminal color


class Flatcat:
    def __init__(self, address, port):
        self.sock = socket.socket()
        self.sock.connect((address, port))
        self.soc = 50
        self.flags = 0
        self.cycle = 0
        self.paused = 0

    def parse_message(self, msg):
        li = msg.split("=", 1)
        return li[1]


    def loop(self):
        self.paused = int( self.request_variable("paused") )
        self.soc = float( self.request_variable("SoC") )
        self.flags = int( self.request_variable("flags") )
        self.cycle += 1
        print("{0:3d} SoC={1:4.1f} F={2:016b} en={3}".format(self.cycle, self.soc, self.flags, self.paused))
        time.sleep(1)
        return True


    def request_variable(self, keystr):
        cmd = keystr+"\n"
        self.sock.send(cmd)
        return self.receive_variable()


    def receive_variable(self):
        msg = self.sock.recv(bufsize)
        return self.parse_message(msg)


    def send_variable(self, keystr, value):
        cmd = "{0}={1}\n".format(keystr, value)
        print("sending command: {0}".format(cmd))
        self.sock.send(cmd)


    def exit_server(self):
        cmd = "EXIT\n"
        self.sock.send(cmd)
        self.sock.close()

# end class Flatcat

def main(argv):
    try:
        robot = Flatcat(address, port)
    except:
        print(MSG + "No connection.\n")
        sys.exit()

    result = True
    robot.send_variable("PAU", 1) # disable robot

    time.sleep(1.) 
    #TODO remove this when multiple commands in 1 msg work

    while (result):
        try:
            result = robot.loop()
        except KeyboardInterrupt: # press CTRL + C to exit
            print(MSG + "Ctrl+C, Bye___")
            result = False
        except Exception as e:
            print(e)
            result = False
        
    robot.exit_server()
    sys.exit()


if __name__ == "__main__": main(sys.argv)

