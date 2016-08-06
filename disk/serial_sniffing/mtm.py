 import serial
from time import sleep, datetime

log = open("log{datetime}.txt", "a+")


class MTM(port_a, port_b):
#this class controls a man-in-the-middle attack on the YASNAC

  def __init__(self, port_a, port_b):
    self.yasnac = serial.connect(port= port_a, baudrate=4800,parity=serial.PARITY_EVEN,timeout=0)
    self.floppy = serial.connect(port= port_b, baudrate=4800,parity=serial.PARITY_EVEN,timeout=0)
    sleep(2)
  

  def rx(self, in_com):
    com  = in_com
    packet = ''
    while com.inWaiting():
      r = com.read()
      packet += r # append the character
    print("{0}\r\n".format(packet))
    log.write("{0}\r\n".format(packet))
    return packet
      
  def  tx(self, out_com msg):
    com = out_com
    com.write(msg)

  def relay(self):
    self.tx(floppy, self.rx(yasnac))
    self.tx(yasnac, self.rx(floppy))

# at the end of the file put log.close()
