"""Galaxy Buds2 kafa takibi: kulakligin sensor verisini PC'de okur, opentrack'e gonderir (Windows ve Linux).

Kullanim:  python buds2_headtracking.py AA:BB:CC:DD:EE:FF [--sadece-sag-sol] [--channel N] [--trace] [--seconds N]
Calisirken: r = duz bakarken ortala, Ctrl+C = cik
            Pencere odakta degilken ortalama: Windows'ta Ctrl+Alt+R, Linux'ta `pkill -USR1 -f buds2_headtracking`
            u = duz bakarken bas, 3 saniye icinde basini yukari kaldir (yukari ekseni olculur, kaydedilir)

Protokol GalaxyBudsClient'tan (timschneeb) alindi; sadece 360 Audio mesajlari gonderilir.
"""
import argparse
import ctypes
import json
import math
import os
import signal
import socket
import struct
import sys
import time
import uuid
from pathlib import Path

if os.name == "nt":
    import msvcrt

    def read_key():
        return msvcrt.getwch().lower() if msvcrt.kbhit() else None
else:
    import select
    import termios
    import tty

    def read_key():
        return sys.stdin.read(1).lower() if select.select([sys.stdin], [], [], 0)[0] else None

# Buds+ ve sonrasi modellerin yonetim servisi (GalaxyBudsClient: Uuids.SppNew)
SPP_UUID = uuid.UUID("2e73a4ad-332d-41fc-90e2-16bef06523f2")

SOM, EOM = 0xFD, 0xDD
SET_SPATIAL_AUDIO = 124
SPATIAL_AUDIO_DATA = 194
SPATIAL_AUDIO_CONTROL = 195

CTRL_ATTACH, CTRL_DETACH, CTRL_ATTACH_OK, CTRL_DETACH_OK, CTRL_KEEPALIVE = 0, 1, 2, 3, 4
DATA_GRV, DATA_WEAR_ON, DATA_WEAR_OFF, DATA_GYROCAL, DATA_STUCK = 32, 33, 34, 35, 36

KEEPALIVE_SEC = 2.0
PRINT_SEC = 0.1
OPENTRACK_ADDR = ("127.0.0.1", 4242)  # opentrack girisi "UDP over network": 6 double (x, y, z, yaw, pitch, roll)

CAL_SEC = 3.0
AXES_FILE = Path(__file__).with_name("eksenler.json")  # u olcumu buraya kaydedilir

# Samsung GRV'de dunya X ekseni dikey (yercekimi); kayittan olculdu.
WORLD_UP = (1.0, 0.0, 0.0)
# Basi yukari kaldirma ekseni, kulaklik ekseninde (30 sn hareket kaydindan). u ile yeniden olculur.
LAT_AXIS = (0.99, 0.00, -0.13)

# Ctrl+Alt+R: pencere odakta olmasa da ortalar
MOD_ALT, MOD_CONTROL, MOD_NOREPEAT, VK_R, WM_HOTKEY = 0x1, 0x2, 0x4000, 0x52, 0x0312


def crc16(data):
    # CRC-16/XMODEM: poly 0x1021, init 0
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def encode(msg_id, payload):
    body = bytes([msg_id, *payload])
    size = len(body) + 2  # msg id + payload + crc
    return bytes([SOM]) + struct.pack("<H", size) + body + struct.pack("<H", crc16(body)) + bytes([EOM])


def frames(buf):
    """Tampondaki tam cerceveleri ayiklar: (msg_id, payload, ham cerceve, crc_ok). Yarim kalan bayt buf'ta kalir."""
    while True:
        start = buf.find(SOM)
        if start < 0:
            buf.clear()
            return
        del buf[:start]
        if len(buf) < 3:
            return
        size = struct.unpack_from("<H", buf, 1)[0] & 0x3FF
        total = 3 + size + 1
        if len(buf) < total:
            return
        frame = bytes(buf[:total])
        if size < 3 or frame[-1] != EOM:
            del buf[:1]
            continue
        del buf[:total]
        body = frame[3:1 + size]
        crc_ok = crc16(body) == struct.unpack_from("<H", frame, 1 + size)[0]
        yield body[0], body[1:], frame, crc_ok


def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def q_conj(q):
    x, y, z, w = q
    return -x, -y, -z, w


def normalize(v):
    n = math.sqrt(sum(c * c for c in v)) or 1.0
    return tuple(c / n for c in v)


def rotate(v, q):
    """q v q^-1. Buds2'de dunya vektorunu kulaklik eksenine cevirir; q_conj(q) ile tersi."""
    x, y, z, _ = q_mul(q_mul(q, (*v, 0.0)), q_conj(q))
    return x, y, z


def cross(a, b):
    return a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]


def rotation_axis(d):
    """d donusunun ekseni (aci < 180 olacak yonde); ~15 dereceden az donmusse None."""
    x, y, z, w = d
    if w < 0:
        x, y, z = -x, -y, -z
    return normalize((x, y, z)) if math.sqrt(x * x + y * y + z * z) > 0.13 else None


def load_lat():
    """Yukari kaldirma ekseni: eksenler.json varsa oradan, yoksa varsayilan."""
    try:
        return normalize(json.loads(AXES_FILE.read_text())["pitch"])
    except (FileNotFoundError, KeyError, TypeError):
        return normalize(LAT_AXIS)


def save_lat(lat):
    AXES_FILE.write_text(json.dumps({"pitch": lat}, indent=2))


def head_angles(q, ref, lat):
    """ref'e gore (yaw, pitch) derece: + yaw = sola, + pitch = yukari.
    Burun yonu dunyada izlenir: yaw = yatayda donusu, pitch = ufka gore acisi. Kulagin egikligi etkilemez."""
    nose = normalize(cross(lat, rotate(WORLD_UP, ref)))  # kulaklik ekseninde; ref aninda yatay
    start, now = rotate(nose, q_conj(ref)), rotate(nose, q_conj(q))  # dunya ekseninde
    yaw = math.atan2(start[1] * now[2] - start[2] * now[1], start[1] * now[1] + start[2] * now[2])
    pitch = math.asin(max(-1.0, min(1.0, now[0])))
    return math.degrees(yaw), math.degrees(pitch)


def bar(yaw):
    pos = max(-20, min(20, round(-yaw / 3)))  # +yaw = sola donus
    cells = ["-"] * 41
    cells[20] = "|"
    cells[20 + pos] = "#"
    return "SOL [" + "".join(cells) + "] SAG"


def sdp_rfcomm_channel(mac, service_uuid):
    """Linux: servisin RFCOMM kanalini SDP ServiceSearchAttributeRequest ile bulur."""
    pattern = b"\x35\x11\x1c" + service_uuid.bytes  # UUID128 iceren dizi
    attrs = b"\x35\x05\x0a\x00\x00\xff\xff"  # 0x0000-0xFFFF tum ozellikler
    params = pattern + b"\xff\xff" + attrs + b"\x00"  # en fazla 0xFFFF bayt, devam yok
    with socket.socket(socket.AF_BLUETOOTH, socket.SOCK_SEQPACKET, socket.BTPROTO_L2CAP) as sdp:
        sdp.settimeout(5)
        sdp.connect((mac, 1))  # SDP = PSM 1
        sdp.send(struct.pack(">BHH", 0x06, 1, len(params)) + params)
        resp = sdp.recv(4096)
    # ProtocolDescriptorList icinde RFCOMM UUID16 (0x0003) ve ardindan uint8 kanal: 19 00 03 08 <kanal>
    i = resp.find(b"\x19\x00\x03\x08")
    if resp[:1] != b"\x07" or i < 0:
        raise OSError(f"SDP'de {service_uuid} bulunamadi. Kanali --channel ile verin (sdptool browse {mac}).")
    return resp[i + 4]


def connect(mac, channel=None):
    sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    if os.name != "nt" or channel:
        sock.connect((mac, channel or sdp_rfcomm_channel(mac, SPP_UUID)))
        return sock
    # Python kanal numarasi istiyor; SOCKADDR_BTH'ye UUID verip port=0 birakinca Windows kanali SDP ile kendisi bulur.
    ws2 = ctypes.WinDLL("ws2_32")
    ws2.connect.argtypes = [ctypes.c_size_t, ctypes.c_char_p, ctypes.c_int]
    addr = struct.pack("<HQ16sI", socket.AF_BLUETOOTH, int(mac.replace(":", ""), 16), SPP_UUID.bytes_le, 0)
    if ws2.connect(sock.fileno(), addr, len(addr)) != 0:
        err = ws2.WSAGetLastError()
        sock.close()
        raise OSError(err, f"Baglanti kurulamadi (WSA hata {err}). Kulaklik PC'ye bagli mi?")
    return sock


def global_recenter():
    """Pencere odakta degilken ortalama tetikleyicisi: (istendi_mi(), kapat(), nasil tetiklenir veya None)."""
    if os.name == "nt":
        from ctypes import wintypes
        user32 = ctypes.WinDLL("user32")
        ok = user32.RegisterHotKey(None, 1, MOD_CONTROL | MOD_ALT | MOD_NOREPEAT, VK_R)
        msg = wintypes.MSG()

        def requested():
            return bool(user32.PeekMessageW(ctypes.byref(msg), None, WM_HOTKEY, WM_HOTKEY, 1))

        def close():
            if ok:
                user32.UnregisterHotKey(None, 1)

        return requested, close, "Ctrl+Alt+R" if ok else None

    flag = []
    signal.signal(signal.SIGUSR1, lambda *_: flag.append(1))

    def requested():
        hit = bool(flag)
        flag.clear()
        return hit

    return requested, lambda: None, "pkill -USR1 -f buds2_headtracking"


def main():
    parser = argparse.ArgumentParser(description="Galaxy Buds2 kafa takibi")
    parser.add_argument("mac", help="kulakligin MAC adresi, or. AA:BB:CC:DD:EE:FF")
    parser.add_argument("--sadece-sag-sol", action="store_true", help="asagi/yukariyi gonderme (pitch hep 0)")
    parser.add_argument("--channel", type=int, help="RFCOMM kanali; normalde otomatik bulunur")
    parser.add_argument("--trace", action="store_true", help="gelen/giden tum cerceveleri hex yaz")
    parser.add_argument("--seconds", type=float, default=0, help="N saniye sonra cik (0 = sonsuz)")
    args = parser.parse_args()

    sock = connect(args.mac, args.channel)
    lat = load_lat()
    recenter_requested, close_recenter, recenter_how = global_recenter()
    print("Baglandi. r = ortala, u = yukari eksenini olc, Ctrl+C = cik")
    if recenter_how:
        print(f"Oyundayken ortalamak icin: {recenter_how}")
    else:
        print("Uyari: Ctrl+Alt+R baska bir programda kayitli; ortalamak icin bu pencerede r'ye bas.")
    term = None
    if os.name != "nt" and sys.stdin.isatty():
        term = termios.tcgetattr(sys.stdin)  # tuslari Enter beklemeden okumak icin
        tty.setcbreak(sys.stdin)
    mode = " (sadece sag/sol)" if args.sadece_sag_sol else ""
    print(f"Aci opentrack'e gonderiliyor: UDP {OPENTRACK_ADDR[0]}:{OPENTRACK_ADDR[1]}{mode}")
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(msg_id, payload):
        frame = encode(msg_id, payload)
        if args.trace:
            print(f">> {msg_id:3d}  {frame.hex(' ')}")
        sock.sendall(frame)

    send(SET_SPATIAL_AUDIO, [1])
    send(SPATIAL_AUDIO_CONTROL, [CTRL_ATTACH])
    sock.settimeout(0.2)

    buf = bytearray()
    ref = last_q = None
    cal = None  # u olcumu suruyorsa: {"until", "best"}; best = baslangica gore en buyuk donus
    started = last_keepalive = time.monotonic()
    last_print = 0.0
    try:
        while not args.seconds or time.monotonic() - started < args.seconds:
            now = time.monotonic()
            if now - last_keepalive >= KEEPALIVE_SEC:
                send(SPATIAL_AUDIO_CONTROL, [CTRL_KEEPALIVE])
                last_keepalive = now
            if recenter_requested() and ref:
                ref = last_q
                print("Ortalandi.")
            key = read_key()
            if key and ref:
                if key == "r":
                    ref = last_q
                    print("Ortalandi.")
                elif key == "u":
                    ref = last_q  # basarken duz bakiliyor: hem ortala hem olcumun baslangici
                    cal = {"until": now + CAL_SEC, "best": (0.0, 0.0, 0.0, 1.0)}
                    print(f"{CAL_SEC:.0f} saniye icinde basini YUKARI kaldir...")
            try:
                chunk = sock.recv(1024)
            except TimeoutError:
                continue
            if not chunk:
                print("Kulaklik baglantiyi kapatti.")
                break
            buf += chunk
            for msg_id, payload, frame, crc_ok in frames(buf):
                if args.trace:
                    print(f"<< {msg_id:3d}  {frame.hex(' ')}{'' if crc_ok else '  (CRC HATALI)'}")
                if msg_id == SPATIAL_AUDIO_CONTROL and payload:
                    print({CTRL_ATTACH_OK: "Sensor acildi.", CTRL_DETACH_OK: "Sensor kapandi."}
                          .get(payload[0], f"Kontrol yaniti: {payload[0]}"))
                elif msg_id == SPATIAL_AUDIO_DATA and payload:
                    kind, data = payload[0], payload[1:]
                    if kind == DATA_GRV and len(data) >= 8:
                        raw = [v / 10000 for v in struct.unpack_from("<4h", data)]
                        norm = math.sqrt(sum(v * v for v in raw)) or 1.0
                        last_q = tuple(v / norm for v in raw)
                        ref = ref or last_q
                        if cal:
                            d = q_mul(last_q, q_conj(ref))
                            if abs(d[3]) < abs(cal["best"][3]):  # |w| kucukse donus acisi buyuk
                                cal["best"] = d
                            if now >= cal["until"]:
                                measured = rotation_axis(cal["best"])
                                if measured is None:
                                    print("Yukari olcumu olmadi: en az 15 derece kaldirmalisin. Tekrar u'ya bas.")
                                else:
                                    lat = measured
                                    save_lat(lat)
                                    print(f"Yukari ekseni olculdu ve {AXES_FILE.name} dosyasina kaydedildi: "
                                          f"({lat[0]:+.2f}, {lat[1]:+.2f}, {lat[2]:+.2f})")
                                cal = None
                        yaw, pitch = head_angles(last_q, ref, lat)
                        if args.sadece_sag_sol:
                            pitch = 0.0
                        udp.sendto(struct.pack("<6d", 0, 0, 0, -yaw, pitch, 0), OPENTRACK_ADDR)  # opentrack'te + yaw = saga
                        if now - last_print >= PRINT_SEC:
                            extra = f"  ham={[round(v, 4) for v in raw]} norm={norm:.3f}" if args.trace else ""
                            print(f"{bar(yaw)}  yaw={yaw:+6.1f}  pitch={pitch:+6.1f}{extra}")
                            last_print = now
                    elif kind == DATA_WEAR_ON:
                        ref = None  # masadaki yon degil, kulaktaki yon referans olsun
                        print("Kulaklik takildi, ortalandi.")
                    elif kind == DATA_WEAR_OFF:
                        print("Kulaklik cikarildi.")
                    elif kind == DATA_STUCK:
                        print("Uyari: sensor takildi (sensor stuck).")
                    elif args.trace:
                        print(f"   veri tipi {kind}: {data.hex(' ')}")
    except KeyboardInterrupt:
        pass
    finally:
        try:
            send(SPATIAL_AUDIO_CONTROL, [CTRL_DETACH])
            send(SET_SPATIAL_AUDIO, [0])
        except OSError:
            pass
        sock.close()
        udp.close()
        close_recenter()
        if term:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, term)
        print("Cikildi.")


if __name__ == "__main__":
    main()
