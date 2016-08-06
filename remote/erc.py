#!/usr/bin/env python
"""
Library for communicating with a YASNAC ERC-series robot

The erc serial protocol is *very* similar to:
https://en.wikipedia.org/wiki/Binary_Synchronous_Communications
"""
import struct
import sys
import time
import collections
import os

import serial


# general global constants
DEBUG = True

# ERC communication constants
SOH = chr(0x01)  # Start Of Heading: denotes the start of the message heading
STX = chr(0x02)  # Start Of Text: denotes end of heading and beginning of data
ETX = chr(0x03)  # End Of Text: signals that all payload data has been sent
EOT = chr(0x04)  # End Of Transmission: indicates the end of transmission
ENQ = chr(0x05)  # Enquiry: requests a response from the receiving station
DLE = chr(0x10)  # Data Link Escape: modifies the meaning of a subsequent char
NAK = chr(0x15)  # Negative Acknowledge: indicates improper communication
ETB = chr(0x17)  # End of Transmission Block: used in place of ETX to indicate
                 # that there are more message blocks for the current message
ACK0 = DLE + chr(0x30)  # Even acknowledgment
ACK1 = DLE + chr(0x31)  # Odd acknowledgment
WACK = DLE + chr(0x6b)  # Wait acknowledgement
RVI = DLE + chr(0x7c)   # Reverse interrupt
TTD = STX + ENQ  # Temporary transmission delay

CONTROL_CHARS = {
    SOH: "SOH",
    STX: "STX",
    ETX: "ETX",
    EOT: "EOT",
    ENQ: "ENQ",
    DLE: "DLE",
    NAK: "NAK",
    ETB: "ETB",
    ACK0: "ACK0",
    ACK1: "ACK1",
    WACK: "WACK",
    RVI: "RVI",
    TTD: "TTD"
}

TRANSACTIONS = {
    # how we issue commands to the robot
    '01,000': 'command from remote computer',

    # job and special system files - transmission
    '02,001': 'put *.JBI',  # indicates independent job data
    '02,002': 'put *.JBR',  # indicates related (master) job data
    '02,011': 'put WEAV.DAT',  # weave data
    '02,012': 'put TOOL.DAT',  # tool data
    '02,013': 'put UFRAME.DAT',  # user coordinate data
    '02,014': 'put ABSWELD.DAT',  # welder condition data undocumented
    '02,015': 'put CV.DAT',  # conveyer condition data
    '02,016': 'put SENSOR.DAT',  # locus correction condition data
    '02,017': 'put COMARC2.DAT',  # com-arc 2 condition data
    '02,018': 'put PC1PC2.DAT',  # phase comprehension data
    '02,020': 'put POSOUT.DAT',  # unknown, undocumented
    '02,022': 'put RECIPRO.DAT',  # unknown,  undocumented
    '02,023': 'put PALACT.DAT',  # palletizing action data, undocumented
    '02,030': 'put SYSTEM.DAT',  # system data

    # job and special system files - request
    '02,051': 'get *.JBI',  # independent job data
    '02,052': 'get *.JBR',  # related (master) job data
    '02,061': 'get WEAV.DAT',  # weave data
    '02,062': 'get TOOL.DAT',  # tool data
    '02,063': 'get UFRAME.DAT',  # user coordinate data
    '02,064': 'get ABSWELD.DAT',  # welder condition data, undoc'd
    '02,065': 'get CV.DAT',  # conveyer condition
    '02,066': 'get SENSOR.DAT',  # locus correction condition data
    '02,067': 'get COMARC2.DAT',  # COM-ARC2 condition data
    '02,068': 'get PC1PC2.DAT',  # phase comprehension data
    '02,070': 'get POSOUT.DAT',  # unknown, undocumented
    '02,072': 'get RECIPRO.DAT',  # unknown, undocumented
    '02,073': 'get PALACT.DAT',  # palletizing action data, undoc'd.
    '02,080': 'get SYSTEM.DAT',  # system data

    # variable data - transmission
    '03,001': 'put byte',
    '03,002': 'put integer',
    '03,003': 'put double',
    '03,004': 'put real',
    '03,005': 'put position (pulse data)',
    '03,006': 'put position (rectangular data)',
    '03,007': 'put external axis (pulse data)',
    '03,008': 'put external axis (rectangular data)',

    # variable data - request
    '03,051': 'get byte',
    '03,052': 'get integer',
    '03,053': 'get double',
    '03,054': 'get real',
    '03,055': 'get position (pulse data)',
    '03,056': 'get position (rectangular data)',
    '03,057': 'get external axis (pulse data)',
    '03,058': 'get external axis (rectangular data)',

    # job execution response
    '90,000': '0000 or a 4 digit error code, response to a command',
    '90,001': 'data response, variable number of digits/data sent as csv'
}

ERRORS = {
    # 1xxx - command test
    '1010': 'command failure',
    '1011': 'command operand number failure',
    '1012': 'command operand value excessive',
    '1013': 'command operand length failure',

    # 2xxx - command execution mode error
    '2010': 'during robot operation',
    '2020': 'during T-PENDANT',
    '2030': 'during panel HOLD',
    '2040': 'during external HOLD',
    '2050': 'during command HOLD',
    '2060': 'during error alarm',
    '2070': 'in servo OFF or stopping by a panel HOLD',

    # 3xxx - command execution error
    '3010': 'servo power on',
    '3040': 'set home position',
    '3070': 'current position is not input',
    '3080': 'END command of job (except master job)',

    # 4xxx - job registration error
    '4010': 'shortage of memory capacity (job registration)',
    '4012': 'shortage of memory capacity (position data registration)',
    '4020': 'job edit prohibit',
    '4030': 'job of same name exists',
    '4040': 'no desired job',
    '4060': 'set execution',
    '4120': 'position data broken',
    '4130': 'no position data',
    '4150': 'END command of job (except master job)',
    '4170': 'instruction data broken',
    '4190': 'unsuitable characters in job name exist',
    '4200': 'unsuitable characters in job name exist',
    '4230': 'instructions which cannot be used by this system exist',

    # 5xxx - file text error
    '5110': 'instruction syntax error',
    '5120': 'position data fault',
    '5130': 'neither NOP or END exists',
    '5170': 'format error',
    '5180': 'data number is inadequate',
    '5200': 'data range exceeded'
}


Message = collections.namedtuple("Message", 'body header footer')


ControlChar = collections.namedtuple("ControlChar", 'char name')


class InvalidBlockStart(Exception):
    """ The block does not start with SOH nor STX """
    pass


class InvalidBlockNeedMore(Exception):
    """ The block appears to be incomplete """
    pass


class InvalidBlockBody(Exception):
    """ Neither ETX nor ETB could be found & the block is at max length """
    pass


class InvalidBlockChecksum(Exception):
    """ The given checksum value doesn't match the actual checksum value """
    pass


class InvalidTransaction(Exception):
    """ General communications failure, expected IO did not happen """
    def __init__(self, expected_value, received_value):
        super(InvalidTransaction, self).__init__((
            "Invalid transaction; expected:{!r} "
            "received:{!r}").format(
                CONTROL_CHARS.get(expected_value, expected_value),
                CONTROL_CHARS.get(received_value, received_value)))


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


def decode_rstats(rstats):
    """
    Given the result of the RSTATS query command, return a list containing
    string keywords that represent the active bits in the result.
    for example: decode_rstat([2,0]) returns ('')
    """
    bits_0 = ('cycle=step', 'cycle=1 cycle', 'cycle=auto', 'running',
              'in-guard driving', 'undocumented_0_32', 'undocumented_0_64',
              'undocumented_0_128')
    bits_1 = ('panel hold', 'teach-box hold', 'external hold', 'command hold',
              'alarm', 'error', 'servos on', 'undocumented_1_128')
    result = list()
    for byte, names in (int(rstats[0]), bits_0), (int(rstats[1]), bits_1):
        for index, bit_name in enumerate(names):
            if byte & (2 ** index):
                result.append(bit_name)
    return result


def multifind(haystack, needles, start=0, end=None):
    """
    like string.find, except the search term is a list of searches that will
    be tried in sequence. the index of the first found match will be returned.
    if no match is found, -1 will be returned
    """
    if end is None:
        end = len(haystack)

    for index, char in enumerate(haystack):
        # fixme: test this next conditional properly
        if not start <= index <= end:
            continue
        if char in needles:
            return index

    return -1


def checksum(block, stop, start=1):
    """ Return the checksum for the given block """
    return sum([ord(c) for c in block[start:stop]])


def encode(header_code, body, name_block=False):
    """
    return a list of raw block strings which represent the given header_code
    and body
    """
    # return a list of blocks properly formatted for transmission, based on
    # the given header_code and body
    if name_block:
        # if this flag is set, the first line of the body is a filename that
        # needs to be in its own block
        (name, _, body_content) = body.partition('\r')
        body_chunks = [name + '\r'] + list(chunks(body_content, 256))
    else:
        body_chunks = list(chunks(body, 256))
    final_chunk = len(body_chunks) - 1  # -1 because of zero list indexing
    result = list()

    # handle the first block
    block = "".join([SOH, header_code, STX, body_chunks.pop(0),
                     ETB if final_chunk > 0 else ETX])
    block += struct.pack("<H", checksum(block, len(block) + 2))
    result.append(block)
    final_chunk -= 1

    # ...and now the rest
    for index, chunk in enumerate(body_chunks):
        block = "".join([STX, chunk, ETB if index < final_chunk else ETX])
        block += struct.pack("<H", checksum(block, len(block) + 2))
        result.append(block)

    return result


def decode(block):
    """
    return a Message object containing the decoded contents of the given block
    """
    if block.startswith(SOH):
        header_bytes = 8
        header = struct.unpack("c6sc", block[0:header_bytes])[1]
    elif block.startswith(STX):
        header_bytes = 1
        header = struct.unpack("c", block[header_bytes])
    elif block in CONTROL_CHARS:
        raise InvalidBlockStart(
            "Attempt to decode {} control char as block".format(
                CONTROL_CHARS[block]))
    else:
        raise InvalidBlockStart(block.__repr__())

    max_search_length = 256 + header_bytes + 1
    body_end = multifind(block, (ETX, ETB), start=header_bytes,
                         end=max_search_length)
    if body_end < 0:
        if len(block) < max_search_length:
            raise InvalidBlockNeedMore
        else:
            raise InvalidBlockBody

    body = block[header_bytes:body_end]
    block_length = body_end + 3
    footer = struct.unpack("<cH", block[body_end:block_length])
    calculated_checksum = checksum(block, block_length - 2)
    if footer[1] != calculated_checksum:
        raise InvalidBlockChecksum("{!r} != {!r}".format(footer[1],
                                                         calculated_checksum))
    return Message(body, header, footer[0])


def filename_to_rootname(filename):
    """
    Return a filename simplified into its rootname, for example:
    ~/jobs/thing.JBI becomes thing
    """
    return os.path.splitext(os.path.basename(filename))[0]


def namefix(filename, content):
    """
    make sure that if a jobname appears inside a job file, the jobname matches
    the jobs's on disk filename. log any corrections to stdout

    WARNING: side effect: this function enforces \r\n line endings
    """
    expected_jobname = filename_to_rootname(filename)
    expected_entry = "//NAME {}".format(expected_jobname)

    result = []
    for line in content.splitlines():
        if line.startswith("//NAME ") and line != expected_entry:
            result.append(expected_entry)
            warn('Altering job content: "{}" -> "{}"'.format(
                line, expected_entry), force=True)
            continue
        result.append(line)
    return "\r\n".join(result) + "\r\n"


def header_code_lookup(mode, filename):
    """
    return the result of a reverse lookup on the TRANSACTIONS dict, for
    example, given "put" and TOOL.DAT, the result is 02,012
    """
    key = mode + " "
    if filename.endswith('.JBI'):
        key += "*.JBI"
    elif filename.endswith('.JBR'):
        key += "*.JBR"
    else:
        key += os.path.basename(filename)  # keep the filename extension here

    for transaction_code, description in TRANSACTIONS.items():
        if description == key:
            return transaction_code

    raise RuntimeError("Couldn't find code for transaction: {}".format(key))


def header_extension_lookup(header_code):
    """
    Return the filename extension associated with the given header code
    """
    return os.path.splitext(TRANSACTIONS.get(header_code, "UNKNOWN.DAT"))[1]


def test():
    """ temporary development function """
    warn("entering erc comms loop")
    robot = ERC()
    robot.loop()


class ERC(object):
    """ Interface to the yasnac ERC series robots """
    handlers = None
    link = None
    ack_bit = False

    def __init__(self):
        self.handlers = dict({
            # Incoming files
            '02,001': self.handle_incoming_file,
            '02,002': self.handle_incoming_file,
            '02,011': self.handle_incoming_file,
            '02,012': self.handle_incoming_file,
            '02,013': self.handle_incoming_file,
            '02,014': self.handle_incoming_file,
            '02,015': self.handle_incoming_file,
            '02,016': self.handle_incoming_file,
            '02,017': self.handle_incoming_file,
            '02,018': self.handle_incoming_file,
            '02,020': self.handle_incoming_file,
            '02,022': self.handle_incoming_file,
            '02,023': self.handle_incoming_file,
            '02,030': self.handle_incoming_file,
            # Outgoing files
            '02,051': self.handle_file_request,
            '02,052': self.handle_file_request,
            '02,061': self.handle_file_request,
            '02,062': self.handle_file_request,
            '02,063': self.handle_file_request,
            '02,064': self.handle_file_request,
            '02,065': self.handle_file_request,
            '02,066': self.handle_file_request,
            '02,067': self.handle_file_request,
            '02,068': self.handle_file_request,
            '02,070': self.handle_file_request,
            '02,072': self.handle_file_request,
            '02,073': self.handle_file_request,
            '02,080': self.handle_file_request,
        })
        self.link = serial.Serial(port='/dev/ttyS0',
                                  baudrate=9600,
                                  bytesize=8,
                                  parity=serial.PARITY_EVEN,
                                  stopbits=serial.STOPBITS_ONE,
                                  timeout=None)
        self.ack_bit = False

    def raw_read(self):
        """ Return the contents of incoming raw data on the serial port """
        input_buffer = []
        while not self.link.inWaiting():
            time.sleep(0.05)
        while self.link.inWaiting():
            input_buffer.append(self.link.read(size=self.link.inWaiting()))
            time.sleep(0.015)
        result = "".join(input_buffer)
        warn("raw_read {} bytes: {!r}".format(len(result), result))
        return result

    def raw_write(self, message):
        """ Send raw data on the serial port """
        self.link.write(message)
        warn("raw_write {} bytes: {!r}".format(len(message), message))

    def current_ack(self):
        """ Return the appropriate ACK message, flip self.ack_bit """
        result = ACK1 if self.ack_bit else ACK0
        self.ack_bit = not self.ack_bit
        return result

    def send_ack(self):
        """
        Send the appropriate ACK message; it alternates between ACK0 and ACK1
        """
        self.raw_write(self.current_ack())

    def send_eot(self):
        """ send an EOT control character, which resets the ACK bit """
        self.raw_write(EOT)
        self.ack_bit = False

    def receive_eot(self, read_from_wire=True):
        """ receive an EOT control character, which resets the ACK bit """
        if read_from_wire:
            # There should be an EOT on the wire. Drain it.
            raw_block = self.raw_read()
            if raw_block != EOT:
                raise InvalidTransaction("EOT", raw_block)
        self.ack_bit = False

    def send_handshake(self):
        """ Ping the robot """
        # send ENQ then listen for an ACK0/ACK1
        self.raw_write(ENQ)
        expected_reply = self.current_ack()
        raw_block = self.raw_read()
        if raw_block != expected_reply:
            raise InvalidTransaction(expected_reply, raw_block)
        return True

    def receive_handshake(self):
        """ Ping the robot """
        expected_input = ENQ
        raw_block = self.raw_read()
        if raw_block != expected_input:
            raise InvalidTransaction(expected_input, raw_block)
        self.send_ack()
        return True

    def confirmed_write(self, block):
        """ Send the given block, wait for the appropriate ack """
        confirmed = False
        while not confirmed:
            self.raw_write(block)
            expected_ack = self.current_ack()
            raw_block = self.raw_read()
            if raw_block == expected_ack:
                confirmed = True
            else:
                warn("wrong ack? will retry.; got:{!r} expected:{!r}".format(
                    raw_block, expected_ack))
        return confirmed

    def short_message(self, header, body, autofix=True):
        """
        Handshake, send a single-block message, send EOT. Useful for transfer
        confirmations and error reports
        """
        if autofix and not body.endswith("\r"):
            body += "\r"
        self.send_handshake()
        self.confirmed_write(encode(header, body)[0])
        self.send_eot()

    def handle_incoming_file(self, message, confirm=True):
        """
        Handle incoming file transfers;
        - save the file to disk
        - send a properly formatted reply message to the yasnac
        """
        (name, _, content) = message.body.partition('\r')
        filename = name + header_extension_lookup(message.header)
        # fixme: safety-check the filename

        # Write the file
        with open(filename, "w") as fileout:
            fileout.write(content)

        if confirm:
            # Now we send back the higher-level transfer confirmation...
            self.short_message("90,000", "0000")

        return filename

    def handle_file_request(self, message):
        """
        Handle a  a file request from the ERC system
        """
        requested_name = message.body.strip()
        filename = requested_name + header_extension_lookup(message.header)
        # fixme: safety-check the filename

        if not os.path.exists(filename):
            log('ERC requested nonexistant file: ' + filename)
            self.short_message("90,000", "4040")
            return

        self.put_file(filename, confirm=False)

        return filename

    def get_file(self, filename, header=None):
        """ Request file data from the ERC """
        if not header:
            header = header_code_lookup("get", filename)
        self.short_message(header, filename_to_rootname(filename))

        # The response is an incoming file transfer
        self.receive_handshake()
        message = self.read_message()
        return self.handle_incoming_file(message, confirm=False)

    def put_file(self, filename, header=None, confirm=True):
        """
        Send the given file to the ERC with an automatically resolved header
        code
        """
        if not header:
            header = header_code_lookup("put", filename)

        rootname = filename_to_rootname(filename)

        with open(filename) as inputfh:
            data = namefix(rootname, inputfh.read())
        payload = rootname + "\r" + data

        self.send_handshake()
        for block in encode(header, payload, name_block=True):
            self.confirmed_write(block)
        self.send_eot()

        if confirm:
            # at this point the ERC will send a confirmation message
            return self.receive_execution_response()

        return True

    def execute_command(self, command_string):
        """ Issue a system control or status read command """
        if not command_string.endswith("\r"):
            command_string += "\r"

        self.short_message("01,000", command_string)
        result = self.receive_execution_response()
        if type(result) != list:
            # this error condition was already warned about, this prevents
            # the error string from being interpreted as a result
            # fixme: to raise or not to raise?
            result = []

        return result

    def receive_execution_response(self):
        """ Receive an incoming 90,00x message, return the contained data """
        result = None

        self.receive_handshake()
        message = self.read_message()
        body = message.body.strip()
        if message.header == "90,001":
            # join all the body lines together, split the result on commas
            result = ",".join(body.splitlines()).split(",")
        elif message.header != "90,000":
            raise InvalidTransaction("confirmation or error message", message)
        elif body != '0000':
            error_string = ERRORS.get(body, "Unknown error " + body)
            result = warn("ERROR from ERC system: {}".format(error_string))

        return result

    def read_message(self, raw_block=None):
        """ Read a complete message from the wire, including multi-block """
        if not raw_block:
            raw_block = self.raw_read()

        if not (raw_block.startswith(SOH) or raw_block.startswith(STX)):
            if raw_block in CONTROL_CHARS:
                return ControlChar(raw_block, CONTROL_CHARS[raw_block])
            else:
                raise InvalidBlockStart("Block starts with invalid sequence: "
                                        + raw_block.__repr__())

        block = decode(raw_block)
        body = block.body
        first_header = block.header
        self.send_ack()

        while block.footer == ETB:
            # ETB means there is more message data in subsequent blocks
            block = decode(self.raw_read())
            body += block.body
            self.send_ack()

        self.receive_eot()

        return Message(body, first_header, block.footer)

    def loop(self):
        """ A continuous event loop for handling ERC serial IO """
        while True:
            raw_block = self.raw_read()

            # raw block handlers
            if raw_block == ENQ:
                self.send_ack()
                continue

            if raw_block == EOT:
                warn("received out-of-sequence EOT")
                self.receive_eot(False)
                continue

            if not raw_block.startswith(SOH):
                warn("No handler for block: " + raw_block.__repr__())
                continue

            # message handlers (block begins with SOH)
            message = self.read_message(raw_block)
            if message.header in self.handlers:
                result = self.handlers[message.header](message)
                log("handled {}, result: {!r}".format(message.header, result))
            else:
                warn("no handler for message " + message.header)
