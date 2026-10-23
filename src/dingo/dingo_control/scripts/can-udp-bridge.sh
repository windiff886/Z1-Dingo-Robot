#!/bin/bash
# Run under can-udp-bridge.service; systemd owns both child processes.
socat udp4-datagram:192.168.131.2:11412,bind=:11412,range=192.168.131.1/24 pty,link=/dev/ttycan0 &

# slcand consumes the PTY created by socat.
while [ ! -e /dev/ttycan0 ]; do
  sleep 0.1
done
slcand -o -c -F -s8 /dev/ttycan0 vcan0 &

# Set link properties after slcand has created the CAN interface.
while ! ip link show dev vcan0 > /dev/null 2>&1; do
  sleep 0.1
done
ip link set dev vcan0 txqueuelen 100
ip link set dev vcan0 up

wait
