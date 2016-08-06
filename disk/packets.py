#!/usr/bin/env python
""" Functions for encoding and decoding packets from the robot """
import struct


class InvalidPacketHeader(Exception):
    pass


class NeedMoreInput(Exception):
    pass


def encode(payload):
    """ Return a string representation of a properly encoded packet """
    length_str = struct.pack("<H2", len(payload))
    checksum = struct.pack("<H2", 65536 -
                           sum([ord(c) for c in length_str + payload]))
    return "\x02{}{}{}".format(length_str, payload, checksum)


def decode(packet):
    """
    Return a decoded packet payload. The packet argument can contain the data
    of multiple packets run together

    packets look like this:
    -----------------------
    2LLPPPP...CC
    0123456.....

    where:
    ------
    2 = the literal byte \x02
    LL = a two byte number indicating the length of the packet
    PPPP... = an LL-length string of bytes
    CC = two bytes of checksum
    """
    if not packet.startswith("\x02"):
        raise InvalidPacketHeader("Unknown packet format")
    length = struct.unpack("<H2", packet[1:3])[0]
    if len(packet) < length + 5:
        raise NeedMoreInput
    payload = packet[1:3+length]  # the initial 2 length bytes are included
    stated_checksum = packet[3+length:5+length]
    calc_checksum = struct.pack("<H2", 65536 - sum([ord(x) for x in payload]))
    if stated_checksum != calc_checksum:
        raise ValueError("Invalid packet! Checksum fail: {} != {}".format(
            stated_checksum.__repr__(), calc_checksum.__repr__()))
    # return a list of packets, excluding the size bytes at the beginning of
    # each payload, and the length of the packet
    return (payload[2:], length + 5)

