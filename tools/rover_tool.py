#!/usr/bin/env python3
"""Host-side tool for the ESP32-S3 rover: BLE pairing, WiFi join, drive + telemetry.

    python rover_tool.py scan
    python rover_tool.py pair --pin 123456 [--address ADDR] [--join]
    python rover_tool.py watch [--host 192.168.4.1]
    python rover_tool.py gui          (needs tkinter)

Pairing protocol: see src/pairing.h.
"""
import argparse
import asyncio
import hashlib
import hmac
import json
import os
import platform
import queue
import subprocess
import sys
import threading
import time

SVC = "7d1c0001-5a3e-4b8f-9a62-3c5e1f0a9b10"
CHAL = "7d1c0002-5a3e-4b8f-9a62-3c5e1f0a9b10"
AUTH = "7d1c0003-5a3e-4b8f-9a62-3c5e1f0a9b10"
STATUS = "7d1c0004-5a3e-4b8f-9a62-3c5e1f0a9b10"
CRED = "7d1c0005-5a3e-4b8f-9a62-3c5e1f0a9b10"
WS_PORT = 81


def ws_url(host: str) -> str:
    """'192.168.4.1' -> ws://192.168.4.1:81/ ; 'host:port' is used as given."""
    return f"ws://{host if ':' in host else f'{host}:{WS_PORT}'}/"


# ------------------------------------------------------------------ protocol
def _hmac(pin: str, label: bytes, ns: bytes, nc: bytes) -> bytes:
    return hmac.new(pin.encode(), label + ns + nc, hashlib.sha256).digest()


def make_auth(pin: str, ns: bytes):
    """Returns (nc, AUTH payload)."""
    nc = os.urandom(16)
    return nc, nc + _hmac(pin, b"auth", ns, nc)


def decrypt_creds(pin: str, ns: bytes, nc: bytes, blob: bytes) -> dict:
    from cryptography.hazmat.primitives.ciphers.aead import AESGCM

    key = _hmac(pin, b"enc", ns, nc)
    iv, ct_tag = blob[:12], blob[12:]
    return json.loads(AESGCM(key).decrypt(iv, ct_tag, b"rover1"))


class PairError(Exception):
    pass


# ----------------------------------------------------------------------- BLE
async def scan(timeout: float = 6.0):
    from bleak import BleakScanner

    found = await BleakScanner.discover(timeout=timeout, return_adv=True)
    rovers = []
    for addr, (dev, adv) in found.items():
        if SVC in [u.lower() for u in (adv.service_uuids or [])]:
            rovers.append((adv.local_name or dev.name or "?", addr, adv.rssi))
    return sorted(rovers, key=lambda r: -r[2])


async def pair(address: str, pin: str) -> dict:
    from bleak import BleakClient

    async with BleakClient(address, timeout=15) as c:
        ns = bytes(await c.read_gatt_char(CHAL))
        nc, payload = make_auth(pin, ns)
        await c.write_gatt_char(AUTH, payload, response=True)
        await asyncio.sleep(0.15)
        st = bytes(await c.read_gatt_char(STATUS))
        code, left = st[0], st[1] if len(st) > 1 else 0
        if code == 1:
            blob = bytes(await c.read_gatt_char(CRED))
            return decrypt_creds(pin, ns, nc, blob)
        if code == 3:
            raise PairError("locked out - too many wrong PINs; press BOOT on the rover to reopen pairing")
        raise PairError(f"wrong PIN ({left} tries left)")


# ---------------------------------------------------------------------- WiFi
def join_wifi(ssid: str, password: str) -> str:
    """Best-effort join of the rover AP. Returns a status message."""
    sysname = platform.system()
    if sysname == "Darwin":
        ports = subprocess.run(["networksetup", "-listallhardwareports"],
                               capture_output=True, text=True).stdout.splitlines()
        dev = next((ports[i + 1].split(":")[1].strip() for i, l in enumerate(ports)
                    if l.startswith("Hardware Port: Wi-Fi") and i + 1 < len(ports)), "en0")
        r = subprocess.run(["networksetup", "-setairportnetwork", dev, ssid, password],
                           capture_output=True, text=True)
        return (r.stdout + r.stderr).strip() or f"joined {ssid} on {dev}"
    if sysname == "Linux":
        r = subprocess.run(["nmcli", "dev", "wifi", "connect", ssid, "password", password],
                           capture_output=True, text=True)
        return (r.stdout + r.stderr).strip()
    return f"Auto-join not supported on {sysname}; join '{ssid}' manually with password '{password}'."


# ----------------------------------------------------------------- telemetry
def fmt_telemetry(d: dict) -> str:
    a, m = d["att"], d["motors"]
    vb = "n/a" if d.get("vbat") is None else f"{d['vbat']:.2f}V"
    return (f"#{d['seq']:<6} pitch {a['pitch']:+6.1f} roll {a['roll']:+6.1f} yaw {a['yaw']:+7.1f} | "
            f"L {m['left']:+.2f} R {m['right']:+.2f} pwm {m['pwm']} | bat {vb} "
            f"{'FAILSAFE ' if d['failsafe'] else ''}clients {d['clients']}")


async def watch(host: str):
    import websockets

    async with websockets.connect(ws_url(host)) as ws:
        async for msg in ws:
            print(fmt_telemetry(json.loads(msg)))


# ----------------------------------------------------------------------- GUI
def run_gui():
    try:
        import tkinter as tk
        from tkinter import ttk
    except ImportError:
        sys.exit("tkinter missing. macOS: brew install python-tk@3.12  (or use the CLI commands)")
    import websockets

    loop = asyncio.new_event_loop()
    threading.Thread(target=loop.run_forever, daemon=True).start()
    ui_q: "queue.Queue" = queue.Queue()

    def bg(coro, done=None):
        """Run coro on the asyncio thread; deliver (result, error) to done on the Tk thread."""
        fut = asyncio.run_coroutine_threadsafe(coro, loop)
        if done:
            def cb(f):
                try:
                    ui_q.put(lambda: done(f.result(), None))
                except Exception as e:  # noqa: BLE001
                    ui_q.put(lambda e=e: done(None, e))
            fut.add_done_callback(cb)

    root = tk.Tk()
    root.title("Rover control")
    root.geometry("760x640")

    creds = {}
    rovers = []
    ws_state = {"ws": None}
    held = {}  # key -> last seen time

    def log(s):
        txt.insert("end", time.strftime("%H:%M:%S ") + s + "\n")
        txt.see("end")

    # --- pairing
    pf = ttk.LabelFrame(root, text="1. Pair over Bluetooth")
    pf.pack(fill="x", padx=8, pady=4)
    lb = tk.Listbox(pf, height=3)
    lb.grid(row=0, column=0, rowspan=2, sticky="nsew", padx=4, pady=4)
    pf.columnconfigure(0, weight=1)
    pin_var = tk.StringVar()

    def do_scan():
        log("scanning...")
        scan_btn.config(state="disabled")

        def done(res, err):
            scan_btn.config(state="normal")
            if err:
                return log(f"scan failed: {err}")
            rovers[:] = res
            lb.delete(0, "end")
            for n, a, r in res:
                lb.insert("end", f"{n}  {a}  {r} dBm")
            if res:
                lb.selection_set(0)
            log(f"found {len(res)} rover(s)")
        bg(scan(), done)

    def do_pair():
        sel = lb.curselection()
        if not sel or len(pin_var.get()) != 6:
            return log("select a rover and enter its 6-digit PIN")
        addr = rovers[sel[0]][1]
        log(f"pairing with {addr}...")

        def done(res, err):
            if err:
                return log(f"pairing failed: {err}")
            creds.update(res)
            cred_var.set(f"SSID: {res['ssid']}    Password: {res['pass']}    IP: {res['ip']}")
            host_var.set(res["ip"])
            log("pairing OK - credentials received")
        bg(pair(addr, pin_var.get()), done)

    scan_btn = ttk.Button(pf, text="Scan", command=do_scan)
    scan_btn.grid(row=0, column=1, padx=4, pady=2)
    ttk.Entry(pf, textvariable=pin_var, width=8).grid(row=1, column=1, padx=4)
    ttk.Button(pf, text="Pair with PIN", command=do_pair).grid(row=1, column=2, padx=4)

    # --- wifi
    wf = ttk.LabelFrame(root, text="2. WiFi")
    wf.pack(fill="x", padx=8, pady=4)
    cred_var = tk.StringVar(value="(not paired)")
    ttk.Label(wf, textvariable=cred_var).pack(side="left", padx=4, pady=4)

    def do_join():
        if not creds:
            return log("pair first")
        log(f"joining {creds['ssid']}...")
        bg(asyncio.to_thread(join_wifi, creds["ssid"], creds["pass"]),
           lambda r, e: log(str(e or r)))

    ttk.Button(wf, text="Join rover WiFi", command=do_join).pack(side="right", padx=4)

    # --- link
    lf = ttk.LabelFrame(root, text="3. Control link")
    lf.pack(fill="x", padx=8, pady=4)
    host_var = tk.StringVar(value="192.168.4.1")
    ttk.Entry(lf, textvariable=host_var, width=16).pack(side="left", padx=4, pady=4)
    tel_var = tk.StringVar(value="disconnected")

    async def ws_main(host):
        try:
            async with websockets.connect(ws_url(host), open_timeout=5) as ws:
                ws_state["ws"] = ws
                ui_q.put(lambda: log("websocket connected"))
                async for msg in ws:
                    d = json.loads(msg)
                    ui_q.put(lambda d=d: tel_var.set(fmt_telemetry(d)))
        except Exception as e:  # noqa: BLE001
            ui_q.put(lambda e=e: log(f"websocket: {e}"))
        finally:
            ws_state["ws"] = None
            ui_q.put(lambda: tel_var.set("disconnected"))

    def do_connect():
        bg(ws_main(host_var.get()))

    def send(obj):
        ws = ws_state["ws"]
        if ws:
            asyncio.run_coroutine_threadsafe(ws.send(json.dumps(obj)), loop)

    ttk.Button(lf, text="Connect", command=do_connect).pack(side="left", padx=4)
    ttk.Button(lf, text="Zero yaw", command=lambda: send({"cmd": "zero_yaw"})).pack(side="left")
    ttk.Button(lf, text="Cal gyro", command=lambda: send({"cmd": "calibrate"})).pack(side="left")
    ttk.Label(root, textvariable=tel_var, font=("Menlo", 10)).pack(fill="x", padx=8)

    # --- drive
    df = ttk.LabelFrame(root, text="4. Drive (WASD / arrows, space = stop)")
    df.pack(fill="x", padx=8, pady=4)
    speed = tk.DoubleVar(value=0.5)
    ttk.Label(df, text="Speed").pack(side="left", padx=4)
    ttk.Scale(df, from_=0.1, to=1.0, variable=speed, length=200).pack(side="left", padx=4, pady=6)
    pad = {"w": (1, 0), "s": (-1, 0), "a": (0, -1), "d": (0, 1)}
    btn_dir = {}

    def drive_cmd():
        now = time.time()
        for k in [k for k, t in held.items() if now - t > 0.3]:
            held.pop(k)
        t = s = 0.0
        for k in held:
            k = {"Up": "w", "Down": "s", "Left": "a", "Right": "d"}.get(k, k)
            if k in pad:
                t += pad[k][0]
                s += pad[k][1]
        if btn_dir:
            t += btn_dir.get("t", 0)
            s += btn_dir.get("s", 0)
        return t, s

    last_active = {"v": False}

    def drive_tick():
        t, s = drive_cmd()
        active = bool(t or s)
        if active:
            l, r = t + s, t - s
            m = max(1.0, abs(l), abs(r))
            send({"cmd": "drive", "left": round(l / m * speed.get(), 3),
                  "right": round(r / m * speed.get(), 3)})
        elif last_active["v"]:
            send({"cmd": "stop"})
        last_active["v"] = active
        root.after(100, drive_tick)

    def hold_btn(text, t, s, col):
        b = ttk.Button(df, text=text, width=4)
        b.grid(row=0, column=col + 1, padx=2)
        b.bind("<ButtonPress-1>", lambda e: btn_dir.update(t=t, s=s))
        b.bind("<ButtonRelease-1>", lambda e: btn_dir.clear())

    for i, (txt_, t, s) in enumerate([("L", 0, -1), ("Fwd", 1, 0), ("Back", -1, 0), ("R", 0, 1)]):
        hold_btn(txt_, t, s, i)
    ttk.Button(df, text="STOP", command=lambda: (held.clear(), btn_dir.clear(), send({"cmd": "stop"}))
               ).grid(row=0, column=6, padx=8)

    root.bind("<KeyPress>", lambda e: held.__setitem__(e.keysym, time.time()) if e.keysym != "space"
              else send({"cmd": "stop"}))
    root.bind("<KeyRelease>", lambda e: root.after(60, lambda: held.pop(e.keysym, None)
                                                   if time.time() - held.get(e.keysym, 0) > 0.05 else None))

    txt = tk.Text(root, height=10)
    txt.pack(fill="both", expand=True, padx=8, pady=4)

    def pump():
        while True:
            try:
                ui_q.get_nowait()()
            except queue.Empty:
                break
        root.after(50, pump)

    pump()
    drive_tick()
    log("Scan, enter the PIN shown on the rover TFT, then Pair.")
    root.protocol("WM_DELETE_WINDOW", lambda: (send({"cmd": "stop"}), root.after(100, root.destroy)))
    root.mainloop()


# ----------------------------------------------------------------------- CLI
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("scan")
    p = sub.add_parser("pair")
    p.add_argument("--pin", required=True)
    p.add_argument("--address")
    p.add_argument("--join", action="store_true", help="also join the rover's WiFi")
    w = sub.add_parser("watch")
    w.add_argument("--host", default="192.168.4.1")
    sub.add_parser("gui")
    a = ap.parse_args()

    if a.cmd == "gui":
        run_gui()
    elif a.cmd == "scan":
        for n, addr, rssi in asyncio.run(scan()):
            print(f"{n}\t{addr}\t{rssi} dBm")
    elif a.cmd == "pair":
        addr = a.address
        if not addr:
            found = asyncio.run(scan())
            if not found:
                sys.exit("no rover found - is the pairing window open (press BOOT)?")
            addr = found[0][1]
            print(f"using {found[0][0]} {addr}")
        try:
            c = asyncio.run(pair(addr, a.pin))
        except PairError as e:
            sys.exit(f"pairing failed: {e}")
        print(json.dumps(c, indent=2))
        if a.join:
            print(join_wifi(c["ssid"], c["pass"]))
    elif a.cmd == "watch":
        try:
            asyncio.run(watch(a.host))
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
