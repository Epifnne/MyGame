"""Minimal minidump parser: extract exception code, faulting address and the
module + offset of the crashing instruction pointer."""
import struct
import sys

path = sys.argv[1]
with open(path, "rb") as f:
    data = f.read()

if data[0:4] != b"MDMP":
    print("not a minidump")
    sys.exit(1)

sig, ver, num_streams, stream_dir_rva, checksum, timestamp, flags = struct.unpack_from(
    "<IIIIIIQ", data, 0)

MINIDUMP_STREAM = {
    3: "ThreadList", 4: "ModuleList", 5: "MemoryList", 6: "Exception",
    7: "SystemInfo", 15: "Memory64List",
}

streams = {}
for i in range(num_streams):
    stype, size, rva = struct.unpack_from("<III", data, stream_dir_rva + i * 12)
    streams[stype] = (rva, size)

# Exception stream
if 6 in streams:
    rva, size = streams[6]
    (thread_id, align) = struct.unpack_from("<II", data, rva)
    # MINIDUMP_EXCEPTION
    (exc_code, exc_flags, exc_record, exc_addr, nparams, align2) = struct.unpack_from(
        "<IIQQII", data, rva + 8)
    print(f"exception code: 0x{exc_code:08X}  faulting IP: 0x{exc_addr:016X}")
    params = struct.unpack_from(f"<{min(nparams,15)}Q", data, rva + 8 + 32)
    if exc_code == 0xC0000005 and nparams >= 2:
        access = {0: "read", 1: "write", 8: "execute"}.get(params[0], str(params[0]))
        print(f"access violation: {access} at address 0x{params[1]:016X}")
    elif exc_code == 0xC0000374:
        print("heap corruption (0xC0000374)")
        if nparams:
            print("params:", [hex(p) for p in params[:4]])

# Module list: map faulting IP to a module
if 4 in streams and 6 in streams:
    rva, size = streams[4]
    (count,) = struct.unpack_from("<I", data, rva)
    off = rva + 4
    exc_ip = struct.unpack_from("<Q", data, streams[6][0] + 8 + 16)[0]
    for i in range(count):
        base, size_m, cs, ts, name_rva = struct.unpack_from("<QI II I".replace(" ", ""), data, off)
        # read module name (minidump string: length + utf16)
        (name_len,) = struct.unpack_from("<I", data, name_rva)
        name = data[name_rva + 4: name_rva + 4 + name_len].decode("utf-16-le", "replace")
        if base <= exc_ip < base + size_m:
            print(f"crash module: {name}  base=0x{base:X}  offset=0x{exc_ip - base:X}")
        off += 108  # MINIDUMP_MODULE size
