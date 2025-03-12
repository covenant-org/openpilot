#!/usr/bin/env python3
import socket
from cereal import messaging


def main():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sm = messaging.sub_sock("modelV2")
    while True:
        data = sm.receive()
        print(data)
        s.sendto(data, ("192.168.100.100", 5000))


if __name__ == "__main__":
    main()
