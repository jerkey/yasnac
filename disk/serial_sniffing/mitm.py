import serial, time, sys

yasnacPort = '/dev/ttyS0'
diskPort = '/dev/ttyUSB0'
baudRate = 4800
parity = serial.PARITY_EVEN
yasnac = serial.Serial(port=yasnacPort, baudrate=baudRate,parity=parity,timeout=0)
disk = serial.Serial(port=diskPort, baudrate=baudRate,parity=parity,timeout=0)

logfileName = 'mitm.log'
logfile = open(logfileName,'w')
header = 'spying on conversation between {0} and {1} at {2} baud.'.format(yasnacPort,diskPort,baudRate)
logfile.write(header)
sys.stdout.write('opened mitm.log\n')
sys.stdout.flush()

time.sleep(1)# wait for the ports to be ready (this is an arbitrary period)
sys.stdout.write('opened serial ports.\n')
sys.stdout.write(header)
sys.stdout.flush()
direction = ''
while True:
  try:
    if yasnac.inWaiting():
      if direction != 'y':
        logfile.write('\nyasnac: ')
        sys.stdout.write('\nyasnac: ')
      direction = 'y'
      d = yasnac.read()
      disk.write(d)
      if (ord(d) < 32) or (ord(d) > 126):
        d = '\\x' + d.encode('hex')
      logfile.write(d)
      logfile.flush()
      if d == '\\x0d':
        d = '\r'
      if d == '\\x0a':
        d = '\n'
      sys.stdout.write(d)
      sys.stdout.flush()
    if disk.inWaiting():
      if direction != 'd':
        logfile.write('\ndisk: ')
        sys.stdout.write('\ndisk: ')
      direction = 'd'
      d = disk.read()
      yasnac.write(d)
      if (ord(d) < 32) or (ord(d) > 126):
        d = '\\x' + d.encode('hex')
      logfile.write(d)
      logfile.flush()
      if d == '\\x0d':
        d = '\r'
      if d == '\\x0a':
        d = '\n'
      sys.stdout.write(d)
      sys.stdout.flush()
  except KeyboardInterrupt:
    print "Bye"
    logfile.close()
    sys.exit()
