#!/usr/bin/python

import sys, socket, time, argparse, logging

# globals
port     = 7332
bufsize  = 4096
MSG = '\033[93m' #orange terminal color
logger = logging.getLogger()

class Flatcat:
    def __init__(self, address, port):
        print(f'{self.__class__.__name__} init socket')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print(f'{self.__class__.__name__} socket connect {address}:{port}')
        self.sock.connect((address, port))
        print(f'{self.__class__.__name__} init params')
        print(f' local socket {self.sock.getsockname()}')
        print(f'remote socket {self.sock.getpeername()}')
        self.soc = 50
        self.flags = 0
        self.cycle = 0
        self.paused = 0

    def parse_message(self, msg):
        li = msg.split("=", 1)
        if len(li)>1:
            return li[1]
        print("message faulted: {0}".format(msg))
        return ""


    def loop(self):
        self.paused = int( self.request_variable("paused") )
        self.soc = float( self.request_variable("SoC") )
        self.flags = int( self.request_variable("flags") )
        self.cycle += 1
        print("{0:3d} SoC={1:4.1f} F={2:016b} pause={3}".format(self.cycle, self.soc, self.flags, self.paused))
        time.sleep(1)
        return True

    def receive(self):
        try:
            msg=self.sock.recv(bufsize)
            msg = msg.decode('utf8')
            print(f"received <{msg}>")
            return msg
        except KeyboardInterrupt:
            print("aborted")
        return ""

    def send(self, msg):
        #print("sending message: <{0}>".format(msg))
        self.sock.send(msg.encode('utf8'))

    def request_variable(self, keystr):
        cmd = keystr+"\n"
        self.send(cmd)
        return self.receive_variable()


    def receive_variable(self):
        msg = self.receive()
        return self.parse_message(msg)


    def receive_ack(self):
        if (self.receive() != "ACK\n"): 
            print("No response")


    def send_variable(self, keystr, value):
        cmd = "{0}={1}\n".format(keystr, value)
        self.send(cmd)
        self.receive_ack()


    def send_command(self, cmd):
        self.send(cmd)
        self.receive_ack()


    def exit_server(self):
        cmd = "EXIT\n"
        self.send(cmd)
        self.sock.close()

# end class Flatcat

def main(argv):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-a', '--address', default="")
    parser.add_argument('-p', '--port', default=port, type=int)
    args = parser.parse_args()

    try:
        robot = Flatcat(args.address, args.port)
    except Exception as e:
        print(f'Exception {e}')
        print(MSG + "No connection.\n")
        sys.exit()

    result = True

    # logger.info(f'sending HELLO')
    print(f'sending HELLO')
    robot.send_command("HELLO")
    # robot.send_variable("PAUSE", 1) # disable robot

    while (result):
        try:
            # result = robot.loop()
            robot.send_command("HELLO")
            time.sleep(1.0)
            result = True
        except KeyboardInterrupt: # press CTRL + C to exit
            print(MSG + "Ctrl+C, Bye___")
            result = False
        except Exception as e:
            print(e)
            result = False
        
    robot.exit_server()
    sys.exit()


if __name__ == "__main__": main(sys.argv)

