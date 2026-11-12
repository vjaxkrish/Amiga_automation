#!/usr/bin/env python3
"""
can_sdo.py - read CANopen object dictionary entries over SDO.

Phase 0. Open question 6: is wheel odometry available?

The motor controller TPDO1s (0x18A-0x18D) carry per-wheel VELOCITY, not
position - decoded 9 Nov 2026 from the 22 Oct capture, bytes 4-5 int16 LE,
r = +/-0.96-0.97 against vehicle speed, no monotonic counter at any offset.

But a TPDO only carries what was mapped to it. The object dictionary may
still hold the position register. This asks.

    python3 can_sdo.py probe                 the useful objects on node 0x0A
    python3 can_sdo.py probe --node 0x0B     ... on another controller
    python3 can_sdo.py scan                  0x1000 on every node 0x0A-0x0D
    python3 can_sdo.py read 0x6064           one object, node 0x0A
    python3 can_sdo.py read 0x6064 --sub 0 --node 0x0C
    python3 can_sdo.py watch 0x6064          poll it - turn a wheel by hand
    python3 can_sdo.py mapping               what each TPDO actually carries
    python3 can_sdo.py sweep --start 0x2000 --end 0x20FF    which objects exist

SDO reads do not command motion and do not change anything. They add a few
frames to the bus and wait for an answer.
"""

import argparse
import socket
import struct
import sys
import time

IFACE = "can0"
CAN_FRAME = struct.Struct("<IB3x8s")

# CANopen SDO: client -> server at 0x600+node, server -> client at 0x580+node
SDO_TX_BASE = 0x600
SDO_RX_BASE = 0x580

CCS_UPLOAD = 0x40          # initiate upload (read) request
CCS_UPLOAD_SEG = 0x60      # upload segment request
SCS_UPLOAD = 0x40          # initiate upload response
SCS_UPLOAD_SEG = 0x00      # upload segment response
SCS_ABORT = 0x80

ABORTS = {
    0x05030000: "toggle bit not alternated",
    0x05040000: "SDO protocol timed out",
    0x05040001: "command specifier not valid",
    0x05040005: "out of memory",
    0x06010000: "unsupported access to an object",
    0x06010001: "attempt to read a write-only object",
    0x06010002: "attempt to write a read-only object",
    0x06020000: "OBJECT DOES NOT EXIST in the dictionary",
    0x06040041: "object cannot be mapped to a PDO",
    0x06040042: "PDO length exceeded",
    0x06040043: "general parameter incompatibility",
    0x06060000: "access failed due to a hardware error",
    0x06070010: "data type does not match, length mismatch",
    0x06090011: "SUB-INDEX DOES NOT EXIST",
    0x06090030: "value range of parameter exceeded",
    0x08000000: "general error",
    0x08000020: "data cannot be transferred or stored",
    0x08000021: "data cannot be transferred, local control",
    0x08000022: "data cannot be transferred, device state",
    0x08000023: "object dictionary not present or dynamic generation failed",
}

# index, sub, label, how to interpret
PROBE = [
    (0x1000, 0, "device type",              "u32"),
    (0x1018, 1, "vendor id",                "u32"),
    (0x1018, 2, "product code",             "u32"),
    (0x1018, 3, "revision number",          "u32"),
    (0x1008, 0, "manufacturer device name", "str"),
    (0x100A, 0, "software version",         "str"),
    (0x6502, 0, "supported drive modes",    "u32"),
    (0x6061, 0, "modes of operation display", "i8"),
    (0x6063, 0, "position actual INTERNAL (increments)", "i32"),
    (0x6064, 0, "POSITION ACTUAL VALUE",    "i32"),
    (0x606C, 0, "velocity actual value",    "i32"),
    (0x6069, 0, "velocity sensor actual value", "i32"),
    (0x6077, 0, "torque actual value",      "i16"),
    (0x6078, 0, "current actual value",     "i16"),
    (0x608F, 1, "encoder increments",       "u32"),
    (0x608F, 2, "motor revolutions",        "u32"),
    (0x6091, 1, "gear ratio: motor revs",   "u32"),
    (0x6091, 2, "gear ratio: shaft revs",   "u32"),
    (0x6092, 1, "feed constant: feed",      "u32"),
    (0x6092, 2, "feed constant: driving revs", "u32"),
]


def open_can(iface=IFACE):
    s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((iface,))
    s.settimeout(0.05)
    return s


def send(sock, cob_id, data8):
    sock.send(CAN_FRAME.pack(cob_id, 8, data8))


def recv_matching(sock, cob_id, timeout=1.0):
    """Next frame with this COB-ID, or None. Other traffic is discarded."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        try:
            raw = sock.recv(16)
        except (socket.timeout, BlockingIOError, OSError):
            continue
        cid, dlc, data = CAN_FRAME.unpack(raw)
        if (cid & 0x1FFFFFFF) == cob_id:
            return data[:8]
    return None


class SdoError(Exception):
    def __init__(self, code):
        self.code = code
        name = ABORTS.get(code, "unknown abort code")
        super().__init__(f"abort 0x{code:08X} - {name}")


class SdoTimeout(Exception):
    pass


def sdo_read(sock, node, index, sub, timeout=1.0):
    """Expedited or segmented upload. Returns raw bytes."""
    tx, rx = SDO_TX_BASE + node, SDO_RX_BASE + node

    req = struct.pack("<BHB4x", CCS_UPLOAD, index, sub)
    send(sock, tx, req)
    rsp = recv_matching(sock, rx, timeout)
    if rsp is None:
        raise SdoTimeout("no response")

    scs = rsp[0] & 0xE0
    if rsp[0] == SCS_ABORT or scs == SCS_ABORT:
        raise SdoError(struct.unpack_from("<I", rsp, 4)[0])
    if scs != SCS_UPLOAD:
        raise SdoTimeout(f"unexpected response 0x{rsp[0]:02X}")

    expedited = bool(rsp[0] & 0x02)
    size_known = bool(rsp[0] & 0x01)

    if expedited:
        n = 4 - ((rsp[0] >> 2) & 0x03) if size_known else 4
        return rsp[4:4 + n]

    # segmented: bytes 4-7 carry the total length
    total = struct.unpack_from("<I", rsp, 4)[0] if size_known else None
    out = b""
    toggle = 0
    for _ in range(64):
        send(sock, tx, struct.pack("<B7x", CCS_UPLOAD_SEG | toggle))
        seg = recv_matching(sock, rx, timeout)
        if seg is None:
            raise SdoTimeout("no response to segment request")
        if seg[0] == SCS_ABORT or (seg[0] & 0xE0) == SCS_ABORT:
            raise SdoError(struct.unpack_from("<I", seg, 4)[0])
        n = 7 - ((seg[0] >> 1) & 0x07)
        out += seg[1:1 + n]
        if seg[0] & 0x01:                       # last segment
            break
        toggle ^= 0x10
    return out[:total] if total else out


def interpret(raw, kind):
    if not raw:
        return "(empty)"
    try:
        if kind == "str":
            return repr(raw.decode("ascii", "replace").rstrip("\x00"))
        if kind == "u32":
            v = int.from_bytes(raw[:4].ljust(4, b"\0"), "little")
            return f"{v} (0x{v:08X})"
        if kind == "i32":
            return str(int.from_bytes(raw[:4].ljust(4, b"\0"), "little",
                                      signed=True))
        if kind == "i16":
            return str(int.from_bytes(raw[:2].ljust(2, b"\0"), "little",
                                      signed=True))
        if kind == "i8":
            return str(int.from_bytes(raw[:1], "little", signed=True))
    except Exception:
        pass
    return raw.hex(" ")


def cmd_probe(args):
    sock = open_can(args.iface)
    node = args.node
    print(f"node 0x{node:02X}  (SDO 0x{SDO_TX_BASE+node:03X} -> "
          f"0x{SDO_RX_BASE+node:03X})\n")

    found = {}
    for index, sub, label, kind in PROBE:
        tag = f"0x{index:04X}:{sub}"
        try:
            raw = sdo_read(sock, node, index, sub, args.timeout)
            val = interpret(raw, kind)
            print(f"  {tag:<12} {label:<38} {val}")
            found[(index, sub)] = raw
        except SdoError as e:
            short = "does not exist" if e.code in (0x06020000, 0x06090011) \
                else str(e)
            print(f"  {tag:<12} {label:<38} -- {short}")
        except SdoTimeout as e:
            print(f"  {tag:<12} {label:<38} -- no answer ({e})")

    print()
    if (0x1000, 0) not in found:
        print("  0x1000 did not answer. Every CANopen node must implement it,")
        print("  so SDO itself is not working here - this says NOTHING about")
        print("  whether a position register exists. Check the node id, that")
        print("  the Amiga is powered, and that can0 is ERROR-ACTIVE.")
        return 1

    pos = found.get((0x6064, 0)) or found.get((0x6063, 0))
    if pos:
        print("  POSITION REGISTER ANSWERS. Run:")
        print(f"     python3 can_sdo.py watch 0x6064 --node 0x{node:02X}")
        print("  and turn that wheel by hand. If the number tracks the wheel,")
        print("  you have real odometry.")
    else:
        print("  No position register. Distance would have to come from")
        print("  integrating the 8.5 Hz velocity field, which drifts.")
    return 0


def cmd_scan(args):
    sock = open_can(args.iface)
    print("node   0x1000 device type        0x6064 position")
    for node in range(0x0A, 0x0E):
        try:
            dt = interpret(sdo_read(sock, node, 0x1000, 0, args.timeout), "u32")
        except (SdoError, SdoTimeout) as e:
            dt = f"-- {e}"
        try:
            pos = interpret(sdo_read(sock, node, 0x6064, 0, args.timeout), "i32")
        except SdoError as e:
            pos = "does not exist" if e.code in (0x06020000, 0x06090011) else "--"
        except SdoTimeout:
            pos = "no answer"
        print(f"0x{node:02X}   {dt:<28} {pos}")
    return 0


def cmd_read(args):
    sock = open_can(args.iface)
    try:
        raw = sdo_read(sock, args.node, args.index, args.sub, args.timeout)
    except (SdoError, SdoTimeout) as e:
        print(f"0x{args.index:04X}:{args.sub} -- {e}")
        return 1
    print(f"0x{args.index:04X}:{args.sub}  raw {raw.hex(' ')}")
    print(f"   as i32 {interpret(raw,'i32'):>14}   "
          f"u32 {interpret(raw,'u32')}")
    if len(raw) >= 2:
        print(f"   as i16 {interpret(raw,'i16'):>14}   "
              f"str {interpret(raw,'str')}")
    return 0


def cmd_watch(args):
    sock = open_can(args.iface)
    print(f"polling 0x{args.index:04X}:{args.sub} on node 0x{args.node:02X}. "
          f"Turn the wheel by hand. Ctrl-C to stop.\n")
    first = None
    last = None
    while True:
        try:
            raw = sdo_read(sock, args.node, args.index, args.sub, args.timeout)
        except (SdoError, SdoTimeout) as e:
            print(f"  -- {e}")
            return 1
        v = int.from_bytes(raw[:4].ljust(4, b"\0"), "little", signed=True)
        if first is None:
            first = v
        delta = "" if last is None else f"  step {v-last:+d}"
        print(f"  {v:>14}   from start {v-first:+d}{delta}")
        last = v
        time.sleep(args.period)


PDO_TYPES = {
    0x08: "u8/i8", 0x10: "u16/i16", 0x20: "u32/i32", 0x40: "u64/i64",
}


def cmd_mapping(args):
    """What each TPDO actually carries, straight from the dictionary.

    This is the definitive decode of the frames already on the bus: whatever
    0x1A00 lists is what those 8 bytes at 0x18A mean, in order.
    """
    sock = open_can(args.iface)
    node = args.node
    print(f"node 0x{node:02X}\n")

    for n in range(4):
        comm, mapi = 0x1800 + n, 0x1A00 + n
        try:
            cob = sdo_read(sock, node, comm, 1, args.timeout)
            cobid = int.from_bytes(cob[:4].ljust(4, b"\0"), "little")
        except (SdoError, SdoTimeout):
            continue
        valid = not (cobid & 0x80000000)
        try:
            ttype = sdo_read(sock, node, comm, 2, args.timeout)[0]
        except (SdoError, SdoTimeout):
            ttype = None

        print(f"TPDO{n+1}  COB-ID 0x{cobid & 0x7FF:03X}"
              f"{'' if valid else '  (DISABLED)'}"
              f"{'' if ttype is None else f'   transmission type {ttype}'}")

        try:
            count = sdo_read(sock, node, mapi, 0, args.timeout)[0]
        except (SdoError, SdoTimeout) as e:
            print(f"   mapping unreadable: {e}\n")
            continue

        bit = 0
        for i in range(1, count + 1):
            try:
                raw = sdo_read(sock, node, mapi, i, args.timeout)
            except (SdoError, SdoTimeout) as e:
                print(f"   entry {i}: {e}")
                continue
            v = int.from_bytes(raw[:4].ljust(4, b"\0"), "little")
            idx, sub, length = (v >> 16) & 0xFFFF, (v >> 8) & 0xFF, v & 0xFF
            lo, hi = bit // 8, (bit + length - 1) // 8
            where = f"byte {lo}" if lo == hi else f"bytes {lo}-{hi}"
            print(f"   {where:<12} 0x{idx:04X}:{sub}  {length:>2} bits"
                  f"   {PDO_TYPES.get(length,'')}")
            bit += length
        print()
    return 0


def cmd_sweep(args):
    """Brute-force which objects exist in a range. Slow but conclusive."""
    sock = open_can(args.iface)
    lo, hi = args.start, args.end
    print(f"node 0x{args.node:02X}: sweeping 0x{lo:04X}-0x{hi:04X} "
          f"({hi-lo+1} objects)\n")
    found = 0
    for index in range(lo, hi + 1):
        try:
            raw = sdo_read(sock, args.node, index, 0, args.timeout)
        except SdoError as e:
            # sub 0 missing but object present -> try sub 1
            if e.code == 0x06090011:
                try:
                    raw = sdo_read(sock, args.node, index, 1, args.timeout)
                except (SdoError, SdoTimeout):
                    continue
            else:
                continue
        except SdoTimeout:
            continue
        found += 1
        print(f"  0x{index:04X}  {raw.hex(' '):<24} "
              f"i32 {interpret(raw,'i32'):>12}  {interpret(raw,'str')}")
    print(f"\n  {found} objects exist in that range")
    return 0


def auto_int(s):
    return int(s, 0)


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--iface", default=IFACE)
    p.add_argument("--node", type=auto_int, default=0x0A,
                   help="CANopen node id, default 0x0A (first motor controller)")
    p.add_argument("--timeout", type=float, default=1.0)
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("probe")
    sub.add_parser("scan")
    sub.add_parser("mapping")

    sw = sub.add_parser("sweep")
    sw.add_argument("--start", type=auto_int, default=0x2000)
    sw.add_argument("--end", type=auto_int, default=0x20FF)

    r = sub.add_parser("read")
    r.add_argument("index", type=auto_int)
    r.add_argument("--sub", type=auto_int, default=0)

    w = sub.add_parser("watch")
    w.add_argument("index", type=auto_int)
    w.add_argument("--sub", type=auto_int, default=0)
    w.add_argument("--period", type=float, default=0.5)

    args = p.parse_args()
    fn = {"probe": cmd_probe, "scan": cmd_scan, "mapping": cmd_mapping,
          "sweep": cmd_sweep, "read": cmd_read, "watch": cmd_watch}[args.cmd]
    try:
        return fn(args)
    except OSError as e:
        print(f"cannot open {args.iface}: {e}")
        print("is can0 up? ip -details link show can0")
        return 1
    except KeyboardInterrupt:
        print("\nstopped.")
        return 0


if __name__ == "__main__":
    sys.exit(main())
