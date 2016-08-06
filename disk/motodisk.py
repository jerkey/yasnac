#!/usr/bin/env python
"""
MotoDisk: a software emulator for the YASNAC FC1 floppy disk drive. This
program allows a YASNAC ERC series robot to have unlimited storage on a host
PC
"""
from time import sleep
import sys
import os
import argparse

import serial

import packets


def log(message):
    """ Print the given message to stdout, flush stdout """
    sys.stdout.write(message + "\n")
    sys.stdout.flush()
    return message


def warn(message, force=False):
    """ Print the given message to stderr, flush stderr """
    if DEBUG or force:
        sys.stderr.write(message + "\n")
        sys.stderr.flush()
    return message


def chunks(iterable, chunksize):
    """
    Yield successive n-sized chunks from the given iterable
    From http://stackoverflow.com/questions/312443
    """
    for i in xrange(0, len(iterable), chunksize):
        yield iterable[i:i+chunksize]


def namefix(filename, filedata):
    """
    make sure that if a jobname appears in the job file, the jobname matches
    the jobs's filename. log any corrections to stdout
    
    WARNING: side effect: this function ensures proper \r\n line endings
    """
    expected_jobname = os.path.splitext(filename)[0]
    expected_entry = "//NAME {}".format(expected_jobname)

    result = []
    for line in filedata.splitlines():
        if line.startswith("//NAME ") and line != expected_entry:
            result.append(expected_entry)
            log('{}: Changing job name from "{}" to "{}"'.format(
                filename, line, expected_entry))
            continue
        result.append(line)
    return "\r\n".join(result) + "\r\n"


class SoftFC1(object):
    """ Emulate the FC1 disk controller """
    com = None
    input_packets = None
    filelist = None
    overwrite = False

    def __init__(self, filelist=None, overwrite=False):
        self.com = serial.Serial(port='/dev/ttyS0', baudrate=4800,
                                 parity=serial.PARITY_EVEN, timeout=None)
        sleep(1)  # wait for the port to be ready (an arbitrary period)
        log("opened serial port")
        self.input_packets = self.input_packet_streamer()  # NOTE: generator
        self.filelist = filelist
        self.overwrite = overwrite

    def raw_read(self):
        """ Return the contents of incoming raw data on the serial port """
        input_buffer = []
        while not self.com.inWaiting():
            sleep(0.05)
        while self.com.inWaiting():
            input_buffer.append(self.com.read(size=self.com.inWaiting()))
            sleep(0.005)
        result = "".join(input_buffer)
        warn("raw_read {} bytes: {}".format(len(result), result.__repr__()))
        return result

    def raw_write(self, message):
        """ Send raw data on the serial port """
        self.com.write(message)
        warn("raw_write {} bytes: {}".format(len(message),
                                             message.__repr__()))

    def write(self, message):
        """ encode and send the given message to the serial port """
        self.raw_write(packets.encode(message))

    def confirmed_write(self, message, limit=10):
        """ send a message, repeating as needed until we get an ack """
        message_received = False
        while not message_received:
            self.write(message)
            packet = next(self.input_packets)
            if packet == "ACK":
                message_received = True
                break
            elif packet == "CAN":
                raise IOError(warn("ERC sent CANcel during confirmed write"))
            else:
                limit -= 1
            if limit < 1:
                raise RuntimeError(warn("Can't to confirm write of {}".format(
                    message.__repr__())))
        warn("Confirmed write of {}".format(message))
        return True

    def input_packet_streamer(self):
        """
        Generator which returns a parsed packet from the buffer, getting more
        data as necessary to complete the packet
        """
        parse_buffer = []
        while True:
            if len(parse_buffer) < 6:
                parse_buffer.extend(self.raw_read())

            try:
                (data, bytes_consumed) = packets.decode("".join(parse_buffer))
                parse_buffer = parse_buffer[bytes_consumed:]
                yield data
            except packets.InvalidPacketHeader:
                # slide out a byte of unusable data
                parse_buffer.pop(0)
            except packets.NeedMoreInput:
                # get some more data from the serial
                parse_buffer.extend(self.raw_read())

    def emulate(self):
        """ Loop, responding to serial requests as needed """

        while True:
            try:
                packet = next(self.input_packets)

                if packet == 'ENQ':
                    warn("Responding to ENQuiry packet")
                    self.write('ACK')
                    continue

                if packet == 'EOT':
                    warn("Received EndOfTransmission packet")
                    continue

                if packet == 'CAN':
                    raise IOError(warn("Received general CANcel packet"))

                if packet == 'ACK':
                    warn("Received unexpected ACKnowledge packet")
                    continue

                if packet == 'LST':
                    warn("Responding to LiST packet")

                    if self.filelist:
                        job_files = ["{:12}".format(filename) for filename
                                     in self.filelist
                                     if os.path.exists(filename)]
                    else:
                        job_files = ["{:12}".format(filename) for filename
                                     in os.listdir('.')
                                     if filename.endswith(".JBI") and
                                     4 < len(filename) < 17]

                    self.confirmed_write("LST{:04}{}".format(
                        len(job_files), "".join(job_files)))
                    self.write("EOF")
                    continue

                if packet == 'DSZ':
                    warn("Responding to DiskSiZe packet")
                    self.confirmed_write("DSZ00729088")
                    self.write("EOF")
                    continue

                if packet.startswith('FRD'):
                    warn("Responding to FileReaD packet")
                    filename = packet[3:].rstrip()
                    if self.filelist and filename not in self.filelist:
                        raise RuntimeError(warn(
                            "The requested filename does not appear on the "
                            "command line", force=True))
                    with open(filename) as inputfh:
                        # autocorrect any filename/jobname discontinuity
                        filedata = namefix(filename, inputfh.read())

                    self.confirmed_write("FSZ{:08}".format(len(filedata)))
                    # send the file in 255 byte blocks, retrying as necessary
                    for chunk in chunks(filedata, 255):
                        self.confirmed_write("FRD" + chunk)
                    self.write("EOF")
                    continue

                if packet.startswith('FWT'):
                    warn("Responding to FileWriTe packet")
                    filename = packet[3:].rstrip()
                    if not self.overwrite and os.path.exists(filename):
                        # we're going to rename TEST.JBI to TEST-1.JBI and
                        # keep incrementing the number until we find one that
                        # doesn't exist
                        original_filename = filename
                        rename_counter = 1
                        while os.path.exists(filename):
                            filename = "{0}-{2}{1}".format(
                                list(os.path.splitext(original_filename)) +
                                [rename_counter])
                            rename_counter += 1
                        warn("Renaming {} to {}".format(original_filename,
                                                        filename))
                    with open(filename, "w") as outputfh:
                        self.write("ACK")
                        while True:
                            packet = next(self.input_packets)
                            if packet.startswith("FWT"):
                                outputfh.write(packet[3:])
                                self.write("ACK")
                                continue
                            if packet == "EOF":
                                self.write("ACK")
                                break
                            warn("Unexpected packet during write: {}".format(
                                packet))
                    continue

                warn("Unhandled packet: {}".format(packet))

            except IOError:
                log("Resetting on CANcel")
                self.write("ACK")


def main():
    """
    primary handler for command-line execution. return an exit status integer
    or a bool type (where True indicates successful exection)
    """
    global DEBUG

    argp = argparse.ArgumentParser(description=(
        "MotoDisk: a software emulator for the YASNAC FC1 floppy disk drive"))
    argp.add_argument('-d', '--debug', action="store_true", help=(
        "enable debugging output"))
    argp.add_argument('-o', '--overwrite', action="store_true", help=(
        "enable existing files to be overwritten by the program"))
    argp.add_argument('file', nargs="*", default=None, help=(
        "optional: if you want only certain file(s) to be available to the "
        "robot, list those files on the command line. For example this "
        "allows you to send just a single file instead of all job (.JBI) "
        "files in the current working directory"))
    args = argp.parse_args()

    DEBUG = args.debug

    disk = SoftFC1(filelist=args.file, overwrite=args.overwrite)
    disk.emulate()

    return True


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        warn("Exiting due to keyboard interrupt (Ctrl-C)", force=True)
