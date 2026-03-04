"""
step_collector_loop.py (Lab 0x05 - Line Follow + Estimator Logger)

This script logs streamed values from your firmware tuning UI.

Primary (always parsed if present):
  - line_err vs time
  - dv_out   vs time

Optional (parsed if your firmware stream includes them):
  - xhat_wL, xhat_wR, xhat_s, xhat_psi
  - x_pos, y_pos, dist

Assumes your UPDATED firmware tuning UI supports:
  - 'h' help
  - 'f' toggle follow_en
  - 'o' set Kp_line / Ki_line
  - 'n' run motors + stream line status indefinitely (stop with 'x')
  - 'x' stop motors/stream
  - 'p' print line status (for sanity checks)

Notes:
  - If your firmware stream includes estimator fields, they will be logged into the CSV
    and extra plots (pose path, heading, distance) will be saved.
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
PORT = "COM9"
BAUD = 115200
TIMEOUT_S = 0.15

LOG_DIR = "Collection Log"
DEBUG = True


# Enable verbose streaming on the device before starting 'n' stream.
# This makes the firmware append posL/posR and yhat_* fields to each streamed line.
AUTO_VERBOSE_STREAM = True
DURATION_S = 9999.0      # not used for streamed 'n' mode; placeholder
SAMPLE_HZ  = 25.0        # nominal parse loop rate (used for sleep)

# --- Startup behavior ---
# The previous version of this script always sent 'n' immediately, which starts
# the motors. For labs where you must calibrate the line sensor first, keep this
# False so you can calibrate before the robot moves.
AUTO_START_MOTORS = False

# Interactive calibration wizard.
# Default key sequence assumes your firmware UI uses: 'c' (cal menu), 'w' (white),
# 'b' (black), 'q' (quit). If your UI uses different keys, change these.
DO_CALIBRATION = True
CAL_KEYS = {
    "menu": "c",
    "white": "w",
    "black": "b",
    "quit": "q",
}

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

# --- State estimator (optional) ---
_re_xhat_wL  = re.compile(r"\bxhat_(?:wL|omegaL)\s*[:=]\s*([-+0-9.eE]+)", re.IGNORECASE)
_re_xhat_wR  = re.compile(r"\bxhat_(?:wR|omegaR)\s*[:=]\s*([-+0-9.eE]+)", re.IGNORECASE)
_re_xhat_s   = re.compile(r"\bxhat_s\s*[:=]\s*([-+0-9.eE]+)", re.IGNORECASE)
_re_xhat_psi = re.compile(r"\bxhat_psi\s*[:=]\s*([-+0-9.eE]+)", re.IGNORECASE)

# Pose keys (new firmware uses x_pos/y_pos; keep x/y for backwards compatibility)
_re_xpos     = re.compile(r"\b(?:x_pos|x)\s*[:=]\s*([-+0-9.eE]+)", re.IGNORECASE)
_re_ypos     = re.compile(r"\b(?:y_pos|y)\s*[:=]\s*([-+0-9.eE]+)", re.IGNORECASE)
_re_dist     = re.compile(r"\bdist(?:_traveled)?\s*[:=]\s*([-+0-9.eE]+)", re.IGNORECASE)

# --- IMU (optional; included in LINE stream) ---
_re_hdg      = re.compile(r"\bhdg\s*[:=]\s*([-+0-9.eE]+)", re.IGNORECASE)
_re_wz       = re.compile(r"\bwz\s*[:=]\s*([-+0-9.eE]+)", re.IGNORECASE)

# --- Encoders (optional; only present when verbose_stream=1) ---
_re_posL     = re.compile(r"\bposL\s*[:=]\s*([-+0-9.eE]+)", re.IGNORECASE)
_re_posR     = re.compile(r"\bposR\s*[:=]\s*([-+0-9.eE]+)", re.IGNORECASE)

# --- Estimated outputs yhat_* (optional; only present when verbose_stream=1 OR EST stream) ---
_re_yhat_sL     = re.compile(r"\byhat_sL\s*[:=]\s*([-+0-9.eE]+)", re.IGNORECASE)
_re_yhat_sR     = re.compile(r"\byhat_sR\s*[:=]\s*([-+0-9.eE]+)", re.IGNORECASE)
_re_yhat_psi    = re.compile(r"\byhat_psi\s*[:=]\s*([-+0-9.eE]+)", re.IGNORECASE)
_re_yhat_psidot = re.compile(r"\byhat_psidot\s*[:=]\s*([-+0-9.eE]+)", re.IGNORECASE)

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

        # --- State estimator (optional) ---
    m = _re_xhat_wL.search(text)
    if m:
        out["xhat_wL"] = float(m.group(1))

    m = _re_xhat_wR.search(text)
    if m:
        out["xhat_wR"] = float(m.group(1))

    m = _re_xhat_s.search(text)
    if m:
        out["xhat_s"] = float(m.group(1))

    m = _re_xhat_psi.search(text)
    if m:
        out["xhat_psi"] = float(m.group(1))

    m = _re_xpos.search(text)
    if m:
        out["x_pos"] = float(m.group(1))

    m = _re_ypos.search(text)
    if m:
        out["y_pos"] = float(m.group(1))

    m = _re_dist.search(text)
    if m:
        out["dist"] = float(m.group(1))

    # IMU (if present)
    m = _re_hdg.search(text)
    if m:
        out["hdg"] = float(m.group(1))

    m = _re_wz.search(text)
    if m:
        out["wz"] = float(m.group(1))

    # Encoders (verbose stream)
    m = _re_posL.search(text)
    if m:
        out["posL"] = float(m.group(1))

    m = _re_posR.search(text)
    if m:
        out["posR"] = float(m.group(1))

    # yhat_* (verbose stream or EST stream)
    m = _re_yhat_sL.search(text)
    if m:
        out["yhat_sL"] = float(m.group(1))

    m = _re_yhat_sR.search(text)
    if m:
        out["yhat_sR"] = float(m.group(1))

    m = _re_yhat_psi.search(text)
    if m:
        out["yhat_psi"] = float(m.group(1))

    m = _re_yhat_psidot.search(text)
    if m:
        out["yhat_psidot"] = float(m.group(1))

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


def run_calibration_interactive(ser: serial.Serial) -> None:
    """Interactive line-sensor calibration (white then black).

    This DOES NOT start motors. It simply sends the UI keystrokes in CAL_KEYS,
    pausing so you can move the robot between surfaces.

    If your firmware uses different keys, edit CAL_KEYS near the top of the file.
    """
    print("\n--- Line Sensor Calibration ---")
    print("This script will guide you through calibration BEFORE starting motors.")
    print("If your UI prompts/keys differ, edit CAL_KEYS in this script.\n")

    # Make sure we're at a prompt and motors are stopped
    try:
        send_key(ser, "x")
        read_until_any(ser, needles=[">:"], max_wait_s=2.0)
    except Exception:
        pass

    try:
        sync_to_tuning_ui(ser)
    except Exception:
        pass

    # Enter calibration menu (if your UI doesn't need this, set CAL_KEYS['menu'] = '' )
    menu_key = (CAL_KEYS.get("menu") or "")[:1]
    if menu_key:
        print(f"Entering calibration menu: sending '{menu_key}'")
        send_key(ser, menu_key)
        read_until_any(ser, needles=[">:"], max_wait_s=4.0)

    # White calibration
    input("Place the line sensor on WHITE background, then press Enter...")
    w_key = (CAL_KEYS.get("white") or "")[:1]
    if not w_key:
        raise RuntimeError("CAL_KEYS['white'] is empty; cannot calibrate white.")
    print(f"Calibrating WHITE: sending '{w_key}'")
    send_key(ser, w_key)
    read_until_any(ser, needles=[">:"], max_wait_s=8.0)

    # Black calibration
    input("Place the line sensor on BLACK line, then press Enter...")
    b_key = (CAL_KEYS.get("black") or "")[:1]
    if not b_key:
        raise RuntimeError("CAL_KEYS['black'] is empty; cannot calibrate black.")
    print(f"Calibrating BLACK: sending '{b_key}'")
    send_key(ser, b_key)
    read_until_any(ser, needles=[">:"], max_wait_s=8.0)

    # Quit calibration menu
    q_key = (CAL_KEYS.get("quit") or "")[:1]
    if q_key:
        print(f"Exiting calibration menu: sending '{q_key}'")
        send_key(ser, q_key)
        read_until_any(ser, needles=[">:"], max_wait_s=4.0)

    print("Calibration complete.\n")


# ------------------------- Output -------------------------

def save_csv(path: str, t_s, err, dv,
             hdg=None, wz=None,
             xhat_s=None, xhat_psi=None, x_pos=None, y_pos=None, dist=None,
             posL=None, posR=None,
             yhat_sL=None, yhat_sR=None, yhat_psi=None, yhat_psidot=None,
             meta=None):
    """Save a CSV. Optional arrays may be None."""
    hdg      = hdg      if hdg      is not None else [float('nan')] * len(t_s)
    wz       = wz       if wz       is not None else [float('nan')] * len(t_s)
    xhat_s   = xhat_s   if xhat_s   is not None else [float('nan')] * len(t_s)
    xhat_psi = xhat_psi if xhat_psi is not None else [float('nan')] * len(t_s)
    x_pos    = x_pos    if x_pos    is not None else [float('nan')] * len(t_s)
    y_pos    = y_pos    if y_pos    is not None else [float('nan')] * len(t_s)
    dist     = dist     if dist     is not None else [float('nan')] * len(t_s)
    posL     = posL     if posL     is not None else [float('nan')] * len(t_s)
    posR     = posR     if posR     is not None else [float('nan')] * len(t_s)
    yhat_sL     = yhat_sL     if yhat_sL     is not None else [float('nan')] * len(t_s)
    yhat_sR     = yhat_sR     if yhat_sR     is not None else [float('nan')] * len(t_s)
    yhat_psi    = yhat_psi    if yhat_psi    is not None else [float('nan')] * len(t_s)
    yhat_psidot = yhat_psidot if yhat_psidot is not None else [float('nan')] * len(t_s)

    with open(path, "w", encoding="utf-8") as f:
        if meta is not None:
            f.write(f"# Kp_line={meta.get('Kp_line', 'N/A')}, Ki_line={meta.get('Ki_line', 'N/A')}\n")
        f.write("Time [s],line_err [QTR units],dv_out [counts/s],hdg [deg],wz [deg/s],xhat_s [mm],xhat_psi [rad],x_pos [mm],y_pos [mm],dist [mm],posL [counts],posR [counts],yhat_sL [mm],yhat_sR [mm],yhat_psi [rad],yhat_psidot [rad/s]\n")
        for t, e, d, h, wz_i, xs, xp, x, y, di, pL, pR, ysL, ysR, ypsi, ypdot in zip(
            t_s, err, dv, hdg, wz, xhat_s, xhat_psi, x_pos, y_pos, dist, posL, posR, yhat_sL, yhat_sR, yhat_psi, yhat_psidot
        ):
            f.write(f"{t:.6f},{e:.6f},{d:.6f},{h:.6f},{wz_i:.6f},{xs:.6f},{xp:.6f},{x:.6f},{y:.6f},{di:.6f},{pL:.0f},{pR:.0f},{ysL:.6f},{ysR:.6f},{ypsi:.6f},{ypdot:.6f}\n")


def save_plot(path: str, t_s, err, dv, xhat_psi=None, x_pos=None, y_pos=None, dist=None, meta=None):
    """Save standard plots (line_err, dv_out) and optional estimator plots."""
    title_suffix = ""
    if meta is not None:
        title_suffix = f"\nKp_line={meta.get('Kp_line','N/A')}  Ki_line={meta.get('Ki_line','N/A')}"

    out_paths = []

    # line_err vs time
    plt.figure()
    plt.plot(t_s, err, label="line_err (centroid error)")
    plt.grid(True)
    plt.xlabel("Time [s]")
    plt.ylabel("line_err [QTR units]")
    plt.title("Line Following Performance (line_err vs time)" + title_suffix)
    plt.legend(loc="best")
    plt.savefig(path, dpi=200, bbox_inches="tight")
    plt.close()
    out_paths.append(path)

    # dv_out vs time
    path_dv = path.replace(".png", "_dv.png")
    plt.figure()
    plt.plot(t_s, dv, label="dv_out (steering correction)")
    plt.grid(True)
    plt.xlabel("Time [s]")
    plt.ylabel("dv_out [counts/s]")
    plt.title("Line Following Control Effort (dv_out vs time)" + title_suffix)
    plt.legend(loc="best")
    plt.savefig(path_dv, dpi=200, bbox_inches="tight")
    plt.close()
    out_paths.append(path_dv)

    # Optional estimator plots
    def _has_finite(arr):
        try:
            return np.isfinite(np.asarray(arr, dtype=float)).any()
        except Exception:
            return False

    if xhat_psi is not None and _has_finite(xhat_psi):
        path_psi = path.replace(".png", "_psi.png")
        plt.figure()
        plt.plot(t_s, xhat_psi, label="xhat_psi (heading)")
        plt.grid(True)
        plt.xlabel("Time [s]")
        plt.ylabel("xhat_psi [rad]")
        plt.title("Estimated Heading (xhat_psi vs time)" + title_suffix)
        plt.legend(loc="best")
        plt.savefig(path_psi, dpi=200, bbox_inches="tight")
        plt.close()
        out_paths.append(path_psi)

    if dist is not None and _has_finite(dist):
        path_dist = path.replace(".png", "_dist.png")
        plt.figure()
        plt.plot(t_s, dist, label="dist (mm)")
        plt.grid(True)
        plt.xlabel("Time [s]")
        plt.ylabel("dist [mm]")
        plt.title("Estimated Distance (dist vs time)" + title_suffix)
        plt.legend(loc="best")
        plt.savefig(path_dist, dpi=200, bbox_inches="tight")
        plt.close()
        out_paths.append(path_dist)

    if x_pos is not None and y_pos is not None and _has_finite(x_pos) and _has_finite(y_pos):
        path_xy = path.replace(".png", "_xy.png")
        plt.figure()
        plt.plot(x_pos, y_pos, label="(x_pos, y_pos)")
        plt.grid(True)
        plt.xlabel("x_pos [mm]")
        plt.ylabel("y_pos [mm]")
        plt.title("Estimated Path (x-y)" + title_suffix)
        plt.legend(loc="best")
        plt.axis("equal")
        plt.savefig(path_xy, dpi=200, bbox_inches="tight")
        plt.close()
        out_paths.append(path_xy)

    return out_paths


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

        if DO_CALIBRATION:
            try:
                run_calibration_interactive(ser)
            except Exception as e:
                print("WARNING: calibration wizard failed:", e)
                print("You can still calibrate manually in the tuning UI, then re-run this script.")

        # Optionally enable verbose streaming so we also get posL/posR and yhat_* fields.
        if AUTO_VERBOSE_STREAM:
            print("Enabling verbose stream on device: sending 'v'")
            send_key(ser, "v")
            # wait for prompt
            read_until_any(ser, needles=[">:"], max_wait_s=4.0)
            time.sleep(0.05)

        if not AUTO_START_MOTORS:
            input("Ready. Press Enter to START motors + stream (sends 'n')...")

        print("Starting motors + stream: sending 'n' to device.")
        send_key(ser, "n")
        time.sleep(0.1)

        times = []
        errs = []
        dvs = []
        xhat_s = []
        xhat_psi = []
        x_pos = []
        y_pos = []
        dist = []
        hdg = []
        wz = []
        posL = []
        posR = []
        yhat_sL = []
        yhat_sR = []
        yhat_psi = []
        yhat_psidot = []
        raw_blocks = []

        meta = {}
        last_vals = {
            "dv_out": float("nan"),
            "line_err": float("nan"),
            "xhat_s": float("nan"),
            "xhat_psi": float("nan"),
            "x_pos": float("nan"),
            "y_pos": float("nan"),
            "dist": float("nan"),
        }

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
                        if not d:
                            continue

                        # Update last-known values
                        for k, v in d.items():
                            last_vals[k] = v

                        # Record a sample whenever we see a streamed measurement
                        if any(k in d for k in ("line_err", "xhat_s", "x_pos", "dist")):
                            t_rel = time.time() - t0
                            times.append(t_rel)
                            errs.append(last_vals.get("line_err", float("nan")))
                            dvs.append(last_vals.get("dv_out", float("nan")))
                            xhat_s.append(last_vals.get("xhat_s", float("nan")))
                            xhat_psi.append(last_vals.get("xhat_psi", float("nan")))
                            x_pos.append(last_vals.get("x_pos", float("nan")))
                            y_pos.append(last_vals.get("y_pos", float("nan")))
                            dist.append(last_vals.get("dist", float("nan")))
                            hdg.append(last_vals.get("hdg", float("nan")))
                            wz.append(last_vals.get("wz", float("nan")))
                            posL.append(last_vals.get("posL", float("nan")))
                            posR.append(last_vals.get("posR", float("nan")))
                            yhat_sL.append(last_vals.get("yhat_sL", float("nan")))
                            yhat_sR.append(last_vals.get("yhat_sR", float("nan")))
                            yhat_psi.append(last_vals.get("yhat_psi", float("nan")))
                            yhat_psidot.append(last_vals.get("yhat_psidot", float("nan")))

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
            print("First sample:", times[0], errs[0], dvs[0], xhat_s[0], x_pos[0], y_pos[0])

        # Quick sanity check: does the estimated (x_pos, y_pos) look like a circle?
        # We do a simple least-squares circle fit (Kasa) and print radius + RMS radial error.
        try:
            xs = np.asarray(x_pos, dtype=float)
            ys = np.asarray(y_pos, dtype=float)
            m = np.isfinite(xs) & np.isfinite(ys)
            if m.sum() >= 10:
                A = np.c_[2*xs[m], 2*ys[m], np.ones(m.sum())]
                b = xs[m]**2 + ys[m]**2
                sol, *_ = np.linalg.lstsq(A, b, rcond=None)
                cx, cy, c0 = sol
                R = float(np.sqrt(max(0.0, c0 + cx*cx + cy*cy)))
                rad = np.sqrt((xs[m]-cx)**2 + (ys[m]-cy)**2)
                rms = float(np.sqrt(np.mean((rad - R)**2)))
                print(f"Circle-fit: center=({cx:.1f},{cy:.1f}) mm, R={R:.1f} mm, RMS radial err={rms:.1f} mm")
            else:
                print("Circle-fit: not enough finite (x_pos,y_pos) samples.")
        except Exception as _e:
            print("Circle-fit: skipped (fit failed).")

        save_csv(csv_path, times, errs, dvs,
                 hdg=hdg, wz=wz,
                 xhat_s=xhat_s, xhat_psi=xhat_psi, x_pos=x_pos, y_pos=y_pos, dist=dist,
                 posL=posL, posR=posR,
                 yhat_sL=yhat_sL, yhat_sR=yhat_sR, yhat_psi=yhat_psi, yhat_psidot=yhat_psidot,
                 meta=meta)
        plot_paths = save_plot(png_path, times, errs, dvs,
                               xhat_psi=xhat_psi, x_pos=x_pos, y_pos=y_pos, dist=dist,
                               meta=meta)
        print("Saved CSV:", csv_path)
        print("Saved plots:")
        for p in plot_paths:
            print("  ", p)

    except KeyboardInterrupt:
        print("KeyboardInterrupt received; attempting to close serial and exit.")
    finally:
        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
