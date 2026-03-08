"""
step_collector_loop.py (Lab 0x05 - Line Follow Logger)

Replaces the old step-response collector.
This script logs LINE FOLLOW performance as:
  line_err vs time
  dv_out vs time

Assumes your UPDATED firmware tuning UI supports:
  - 'h' help
  - 'c' calibration wizard (then 'w' for white, 'b' for black, 'q' quit)
  - 'f' toggle follow_en
  - 'o' set Kp_line / Ki_line
  - 'p' print line status including:
        follow_en: <0/1>
        line_err:  <number>
        dv_out:    <number>

It does NOT require entering the data-collection UI ('g').
"""

import os
import time
from datetime import datetime
import re
import sys
import threading

import serial
import serial.serialutil
import matplotlib.pyplot as plt
import numpy as np


# ------------------------- User Settings -------------------------
PORT = "COM6"
BAUD = 115200
TIMEOUT_S = 0.15

LOG_DIR = "Collection Log"
DEBUG = True

DURATION_S = 9999.0      # not used for streamed 'n' mode; placeholder
SAMPLE_HZ  = 25.0        # nominal parse loop rate (used for sleep)
DO_CALIBRATION = False   # run wizard 'c' -> 'w' -> 'b' -> 'q' on startup
ENABLE_FOLLOW = True     # toggle follow_en to 1 if it's 0 on startup
SET_LINE_GAINS = False   # set True if you want the script to set Kp_line/Ki_line
KPLINE = 0.20
KILINE = 0.00

PROMPTS = (">: ", ">:")  # prompt variants
# ----------------------------------------------------------------


# ------------------------- Helpers -------------------------

def ensure_log_dir() -> str:
    os.makedirs(LOG_DIR, exist_ok=True)
    return LOG_DIR


def timestamp() -> str:
    return datetime.now().strftime("%m_%d_%H_%M_%S")


def _dbg(*args):
    if DEBUG:
        print(*args)


def send_key(ser: serial.Serial, ch: str) -> None:
    """Send a single UI command character (NO newline)."""
    ch = (ch or "")[:1]
    _dbg("TX_KEY:", repr(ch))
    ser.write(ch.encode("utf-8"))
    ser.flush()


def send_number(ser: serial.Serial, value) -> None:
    """Send a numeric entry followed by Enter."""
    s = str(value)
    _dbg("TX_NUM:", repr(s))
    ser.write((s + "\r").encode("utf-8"))
    ser.flush()


def send_newline(ser: serial.Serial, n: int = 1) -> None:
    """Send raw carriage returns to 'poke' the UI."""
    for _ in range(n):
        _dbg("TX_NL:", repr("\\r"))
        ser.write(b"\r")
    ser.flush()


def read_some(ser: serial.Serial, n=512) -> bytes:
    try:
        return ser.read(n)
    except Exception:
        return b""


def drain(ser: serial.Serial, seconds=0.3) -> None:
    t0 = time.time()
    while time.time() - t0 < seconds:
        chunk = read_some(ser, 512)
        if not chunk:
            time.sleep(0.01)


def _text_has_any_prompt(text_lower: str) -> bool:
    return any(p.lower() in text_lower for p in PROMPTS)


def read_until_any(ser: serial.Serial, needles, max_wait_s=6.0):
    """
    Read until ALL `needles` substrings appear (case-insensitive) OR timeout.
    Returns (ok, full_text).
    """
    t0 = time.time()
    buf = bytearray()
    needles_low = [n.lower() for n in needles]

    while time.time() - t0 < max_wait_s:
        chunk = read_some(ser, 512)
        if chunk:
            buf.extend(chunk)
            text = buf.decode("utf-8", errors="replace")
            if DEBUG:
                printable = text.replace("\r", "\\r").replace("\n", "\\n")
                _dbg("RX_TAIL:", printable[-260:])

            low = text.lower()
            if all(n in low for n in needles_low):
                return True, text
        else:
            time.sleep(0.01)

    return False, buf.decode("utf-8", errors="replace")


def sync_to_tuning_ui(ser: serial.Serial, max_wait_s=10.0) -> str:
    """
    Force tuning UI to show menu/prompt by poking newline + 'h'.
    """
    drain(ser, 0.5)

    send_newline(ser, 2)
    time.sleep(0.05)
    send_key(ser, "h")

    ok, text = read_until_any(
        ser,
        needles=["ME 405 Romi Tuning Interface"],
        max_wait_s=max_wait_s,
    )
    if not ok:
        ok2, text2 = read_until_any(ser, needles=[">:"], max_wait_s=4.0)
        if not ok2:
            raise RuntimeError("Could not sync to tuning UI (no banner/prompt).")
        return text2

    if not _text_has_any_prompt(text.lower()):
        okp, textp = read_until_any(ser, needles=[">:"], max_wait_s=4.0)
        if not okp:
            raise RuntimeError("Synced banner but no prompt found.")
        text += "\n" + textp

    return text


# ------------------------- Parsing -------------------------

_re_follow = re.compile(r"follow_en\s*[:=]\s*([01])", re.IGNORECASE)
_re_err    = re.compile(r"line_err\s*[:=]\s*([-+0-9.eE]+)", re.IGNORECASE)

# VERY tolerant dv match: dv_out, dvout, dv, dv_out=, dv=, etc.
_re_dv     = re.compile(r"\bdv(?:_?out)?\s*[:=]\s*([-+0-9.eE]+)", re.IGNORECASE)

_re_kpl    = re.compile(r"Kp_line:\s*([-+0-9.eE]+)", re.IGNORECASE)
_re_kil    = re.compile(r"Ki_line:\s*([-+0-9.eE]+)", re.IGNORECASE)

# Extract per-line samples robustly
_re_line_sample = re.compile(r".*", re.DOTALL)


def parse_line_status(text: str):
    """
    Extract follow_en, line_err, dv_out, Kp_line, Ki_line.
    Returns dict with keys if found.
    """
    out = {}

    m = _re_follow.search(text)
    if m:
        out["follow_en"] = int(m.group(1))

    m = _re_err.search(text)
    if m:
        out["line_err"] = float(m.group(1))

    m = _re_dv.search(text)
    if m:
        out["dv_out"] = float(m.group(1))

    m = _re_kpl.search(text)
    if m:
        out["Kp_line"] = float(m.group(1))

    m = _re_kil.search(text)
    if m:
        out["Ki_line"] = float(m.group(1))

    return out


def get_status_once(ser: serial.Serial) -> dict:
    """
    Press 'p', read until prompt returns, parse out values.
    """
    drain(ser, 0.05)
    send_key(ser, "p")
    ok, txt = read_until_any(ser, needles=[">:"], max_wait_s=4.0)
    return parse_line_status(txt)


def ensure_follow_enabled(ser: serial.Serial) -> None:
    """
    Ensures follow_en is 1 by checking status and toggling with 'f' if needed.
    """
    try:
        sync_to_tuning_ui(ser)
    except Exception:
        pass

    s = get_status_once(ser)
    fe = s.get("follow_en", None)
    print("follow_en (before):", fe)

    if fe is None:
        print("WARNING: Could not parse follow_en. Not toggling.")
        return

    if fe == 0:
        send_key(ser, "f")
        read_until_any(ser, needles=[">:"], max_wait_s=4.0)
        s2 = get_status_once(ser)
        print("follow_en (after):", s2.get("follow_en", None))
    else:
        print("follow_en already enabled.")


# ------------------------- Output -------------------------

def save_csv(path: str, t_s, err, dv, meta=None):
    with open(path, "w", encoding="utf-8") as f:
        if meta is not None:
            f.write(f"# Kp_line={meta.get('Kp_line', 'N/A')}, Ki_line={meta.get('Ki_line', 'N/A')}\n")
        f.write("Time [s],line_err [QTR units],dv_out [counts/s]\n")
        for t, e, d in zip(t_s, err, dv):
            f.write(f"{t:.6f},{e:.6f},{d:.6f}\n")


def save_plot(path: str, t_s, err, dv, meta=None):

    title_suffix = ""
    if meta is not None:
        title_suffix = f"\nKp_line={meta.get('Kp_line','N/A')}  Ki_line={meta.get('Ki_line','N/A')}"

    plt.figure()
    plt.plot(t_s, err, label="line_err (centroid error)")
    plt.grid(True)
    plt.xlabel("Time [s]")
    plt.ylabel("line_err [QTR units]")
    plt.title("Line Following Performance (line_err vs time)" + title_suffix)
    plt.legend(loc="best")
    plt.savefig(path, dpi=200, bbox_inches="tight")
    plt.close()

    path2 = path.replace(".png", "_dv.png")
    plt.figure()
    plt.plot(t_s, dv, label="dv_out (steering correction)")
    plt.grid(True)
    plt.xlabel("Time [s]")
    plt.ylabel("dv_out [counts/s]")
    plt.title("Line Following Control Effort (dv_out vs time)" + title_suffix)
    plt.legend(loc="best")
    plt.savefig(path2, dpi=200, bbox_inches="tight")
    plt.close()
    return path2


# ------------------------- Serial open / reconnect -------------------------

def open_serial_with_retry(port, baud, timeout):
    """Open serial port with a user-prompted retry loop."""
    while True:
        try:
            ser = serial.Serial(port, baud, timeout=timeout)
            print(f"Opened serial {port} @ {baud}")
            return ser
        except (serial.SerialException, OSError) as e:
            print(f"Could not open serial {port}: {e!s}")
            input("Fix device/port and press Enter to retry...")


# ------------------------- Keyboard watcher -------------------------

class KeyboardStopWatcher:
    """Watches stdin for a single-line 'x' command (press Enter after 'x')."""
    def __init__(self):
        self._stop_requested = False
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _reader(self):
        while True:
            try:
                line = sys.stdin.readline()
                if not line:
                    break
                if line.strip().lower() == "x":
                    self._stop_requested = True
                    break
            except Exception:
                break

    def stop_requested(self):
        return self._stop_requested


# ------------------------- Main -------------------------


def main():
    ensure_log_dir()
    tag = timestamp()

    csv_path = os.path.join(LOG_DIR, f"{tag}_line_follow.csv")
    png_path = os.path.join(LOG_DIR, f"{tag}_line_err.png")
    raw_path = os.path.join(LOG_DIR, f"{tag}_raw.txt")

    ser = open_serial_with_retry(PORT, BAUD, timeout=TIMEOUT_S)

    try:
        time.sleep(2.5)
        drain(ser, 1.0)

        try:
            sync_to_tuning_ui(ser)
        except Exception as e:
            _dbg("Sync warning:", e)

        if ENABLE_FOLLOW:
            try:
                ensure_follow_enabled(ser)
            except Exception as e:
                print("WARNING: ensure_follow_enabled failed:", e)

        print("Starting motors + stream: sending 'n' to device.")
        send_key(ser, "n")
        time.sleep(0.1)

        times = []
        errs = []
        dvs = []
        raw_blocks = []

        meta = {}
        last_dv = float("nan")

        kb = KeyboardStopWatcher()
        t0 = time.time()
        print("Running. Type 'x' then Enter in this terminal to stop and save plots.")

        buf = ""
        while True:
            if kb.stop_requested():
                print("Stop requested by user. Sending 'x' to device to stop motors/stream.")
                try:
                    send_key(ser, "x")
                except Exception:
                    pass
                time.sleep(0.2)
                try:
                    tail = ser.read(4096).decode("utf-8", errors="replace")
                    buf += tail
                except Exception:
                    pass
                break

            try:
                chunk = ser.read(512)
                if chunk:
                    s = chunk.decode("utf-8", errors="replace")
                    buf += s
                    raw_blocks.append(s)

                    # Pull meta if present
                    pm = parse_line_status(buf)
                    if "Kp_line" in pm:
                        meta["Kp_line"] = pm["Kp_line"]
                    if "Ki_line" in pm:
                        meta["Ki_line"] = pm["Ki_line"]

                    # Process complete lines; keep the remainder
                    lines = buf.split("\n")
                    buf = lines[-1]
                    for line in lines[:-1]:
                        d = parse_line_status(line)

                        if "dv_out" in d:
                            last_dv = d["dv_out"]

                        if "line_err" in d:
                            t_rel = time.time() - t0
                            times.append(t_rel)
                            errs.append(d["line_err"])
                            dvs.append(last_dv)

                else:
                    time.sleep(1.0 / max(1.0, SAMPLE_HZ))

            except (serial.serialutil.SerialException, OSError) as e:
                print("Serial error detected during streaming:", e)
                try:
                    ser.close()
                except Exception:
                    pass
                print("Attempting to re-open serial port...")
                ser = open_serial_with_retry(PORT, BAUD, timeout=TIMEOUT_S)
                try:
                    sync_to_tuning_ui(ser)
                except Exception:
                    pass

        print("Saving logs...")
        with open(raw_path, "w", encoding="utf-8") as f:
            for b in raw_blocks:
                f.write(b)

        print("Samples collected:", len(times))
        if len(times) > 0:
            print("First sample:", times[0], errs[0], dvs[0])

        save_csv(csv_path, times, errs, dvs, meta=meta)
        dv_png = save_plot(png_path, times, errs, dvs, meta=meta)
        print("Saved CSV:", csv_path)
        print("Saved plots:", png_path, dv_png)

    except KeyboardInterrupt:
        print("KeyboardInterrupt received; attempting to close serial and exit.")
    finally:
        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()