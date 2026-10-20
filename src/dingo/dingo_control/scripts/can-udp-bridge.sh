#!/bin/bash
# Keep both bridge processes in the foreground service's process group.
set -Eeuo pipefail

serial_link=${DINGO_BRIDGE_SERIAL_LINK:-/dev/ttycan0}
can_interface=${DINGO_CAN_INTERFACE:-vcan0}
lock_file=${DINGO_BRIDGE_LOCK_FILE:-/run/lock/dingo-can-udp-bridge.lock}
peer_ip=${DINGO_MCU_IP:-192.168.131.2}
udp_port=${DINGO_UDP_PORT:-11412}
peer_range=${DINGO_PEER_RANGE:-192.168.131.1/24}
socat_pid=
slcand_pid=
owns_serial=false
owns_can=false

fail() {
  echo "dingo CAN/UDP bridge: $*" >&2
  exit 1
}

cleanup() {
  trap - EXIT
  if [[ "$owns_can" == true ]]; then
    ip link set dev "$can_interface" down 2>/dev/null || true
  fi
  for bridge_pid in "$slcand_pid" "$socat_pid"; do
    if [[ -n "$bridge_pid" ]]; then
      kill "$bridge_pid" 2>/dev/null || true
    fi
  done
  for bridge_pid in "$slcand_pid" "$socat_pid"; do
    if [[ -n "$bridge_pid" ]]; then
      wait "$bridge_pid" 2>/dev/null || true
    fi
  done
  if [[ "$owns_serial" == true && -L "$serial_link" ]]; then
    rm -f -- "$serial_link"
  fi
}

for bridge_command in socat slcand ip flock; do
  command -v "$bridge_command" >/dev/null || fail "missing command: $bridge_command"
done

# Do not remove the lock file: unlinking it would let another process lock a
# different inode while this service's descendants still own the first lock.
exec 9>"$lock_file"
flock -n 9 || fail "another bridge instance holds $lock_file"
[[ ! -e "$serial_link" && ! -L "$serial_link" ]] || fail "serial link already exists: $serial_link"
if ip link show dev "$can_interface" >/dev/null 2>&1; then
  fail "CAN interface already exists: $can_interface; check the existing bridge"
fi

trap cleanup EXIT
trap 'exit 0' TERM INT
socat "udp4-datagram:$peer_ip:$udp_port,bind=:$udp_port,range=$peer_range" \
  "pty,link=$serial_link" &
socat_pid=$!
owns_serial=true

for ((attempt = 0; attempt < 50; attempt++)); do
  kill -0 "$socat_pid" 2>/dev/null || fail "socat exited before the serial link was ready"
  [[ -e "$serial_link" ]] && break
  sleep 0.1
done
[[ -e "$serial_link" ]] || fail "timed out waiting for $serial_link"

# -F prevents slcand from daemonizing, so its actual lifetime is supervised.
slcand -o -c -F -s8 "$serial_link" "$can_interface" &
slcand_pid=$!
for ((attempt = 0; attempt < 50; attempt++)); do
  kill -0 "$socat_pid" 2>/dev/null || fail "socat exited during CAN setup"
  kill -0 "$slcand_pid" 2>/dev/null || fail "slcand exited during CAN setup"
  if ip link show dev "$can_interface" >/dev/null 2>&1; then
    owns_can=true
    break
  fi
  sleep 0.1
done
[[ "$owns_can" == true ]] || fail "timed out waiting for $can_interface"
ip link set dev "$can_interface" txqueuelen 100
ip link set dev "$can_interface" up
echo "dingo CAN/UDP bridge: $can_interface ready"

# Any unexpected child exit, including exit code 0, tears down the pair and
# returns failure to systemd. The signal trap handles intentional shutdown.
if wait -n "$socat_pid" "$slcand_pid"; then
  fail "a bridge process exited unexpectedly"
else
  fail "a bridge process failed"
fi
