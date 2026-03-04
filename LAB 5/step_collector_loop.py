"""
step_collector_loop.py (Lab 0x05 - Line Follow Logger)

Replaces the old step-response collector.
This script logs LINE FOLLOW performance as:
  line_err vs time
  dv_out vs time (optional)

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

import serial
import matplotlib.pyplot as plt
import numpy as np


# ------------------------- User Settings -------------------------
PORT = "COM9"
BAUD = 115200
TIMEOUT_S = 0.15

LOG_DIR = "Collection Log"
DEBUG = True

DURATION_S = 8.0        # how long to log
SAMPLE_HZ  = 25.0       # how often to press 'p'
DO_CALIBRATION = True   # run wizard 'c' -> 'w' -> 'b' -> 'q'
ENABLE_FOLLOW = True    # toggle follow_en to 1 if it's 0
SET_LINE_GAINS = False  # set True if you want the script to set Kp_line/Ki_line
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
    return ser.read(n)


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
        # fallback: at least get a prompt
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

_re_follow = re.compile(r"follow_en:\s*([01])", re.IGNORECASE)
_re_err    = re.compile(r"line_err:\s*([-+0-9.eE]+)", re.IGNORECASE)
_re_dv     = re.compile(r"dv_out:\s*([-+0-9.eE]+)", re.IGNORECASE)


def parse_line_status(text: str):
    """
    Extract follow_en, line_err, dv_out from the printed status block.
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

    return out


# ------------------------- UI Actions -------------------------

def set_line_gains(ser: serial.Serial, kp_line: float, ki_line: float) -> None:
    """
    Uses 'o' to set Kp_line and Ki_line.
    """
    sync_to_tuning_ui(ser)
    drain(ser, 0.2)

    send_key(ser, "o")
    ok, _ = read_until_any(ser, needles=["Enter line Kp_line"], max_wait_s=6.0)
    if not ok:
        # Some versions may print slightly different text; fallback
        ok, _ = read_until_any(ser, needles=["Enter line"], max_wait_s=6.0)
        if not ok:
            raise RuntimeError("Timed out waiting for line Kp prompt.")

    send_number(ser, kp_line)

    ok, _ = read_until_any(ser, needles=["Enter line Ki_line"], max_wait_s=6.0)
    if not ok:
        ok, _ = read_until_any(ser, needles=["Ki_line"], max_wait_s=6.0)
        if not ok:
            raise RuntimeError("Timed out waiting for line Ki prompt.")

    send_number(ser, ki_line)

    ok, _ = read_until_any(ser, needles=[">:"], max_wait_s=6.0)
    if not ok:
        raise RuntimeError("Timed out returning to tuning prompt after line gains.")


def calibration_wizard(ser: serial.Serial) -> None:
    """
    Runs: c -> w -> b -> q
    You must position the robot appropriately when prompted.
    """
    sync_to_tuning_ui(ser)
    drain(ser, 0.2)

    print("\nCALIBRATION:")
    print("1) Put sensors over WHITE background. Press Enter here when ready...")
    input()

    send_key(ser, "c")
    ok, txt = read_until_any(ser, needles=["Calibration wizard"], max_wait_s=6.0)
    if not ok:
        # Still try
        _dbg("Calibration banner not seen; proceeding anyway.")

    send_key(ser, "w")
    ok, _ = read_until_any(ser, needles=["Calibrating WHITE"], max_wait_s=6.0)
    if not ok:
        _dbg("Did not see 'Calibrating WHITE', continuing.")

    # Give MCU time to do samples
    time.sleep(0.8)

    print("2) Put sensors over BLACK line. Press Enter here when ready...")
    input()

    send_key(ser, "b")
    ok, _ = read_until_any(ser, needles=["Calibrating BLACK"], max_wait_s=6.0)
    if not ok:
        _dbg("Did not see 'Calibrating BLACK', continuing.")

    time.sleep(0.8)

    send_key(ser, "q")
    read_until_any(ser, needles=[">:"], max_wait_s=6.0)
    print("Calibration sequence sent.\n")


def get_status_once(ser: serial.Serial) -> dict:
    """
    Press 'p', read until prompt returns, parse out values.
    """
    drain(ser, 0.05)
    send_key(ser, "p")
    ok, txt = read_until_any(ser, needles=[">:"], max_wait_s=4.0)
    if not ok:
        txt = txt  # parse what we got
    return parse_line_status(txt)


def ensure_follow_enabled(ser: serial.Serial) -> None:
    """
    Ensures follow_en is 1 by checking status and toggling with 'f' if needed.
    """
    sync_to_tuning_ui(ser)
    s = get_status_once(ser)
    fe = s.get("follow_en", None)

    if fe is None:
        print("WARNING: Could not parse follow_en from status. Trying toggle anyway...")
        send_key(ser, "f")
        read_until_any(ser, needles=[">:"], max_wait_s=4.0)
        return

    if fe == 0:
        send_key(ser, "f")
        read_until_any(ser, needles=[">:"], max_wait_s=4.0)
        s2 = get_status_once(ser)
        print("follow_en now:", s2.get("follow_en", "?"))
    else:
        print("follow_en already enabled.")


# ------------------------- Output -------------------------

def save_csv(path: str, t_s, err, dv):
    with open(path, "w", encoding="utf-8") as f:
        f.write("Time [s],line_err [QTR units],dv_out [counts/s]\n")
        for t, e, d in zip(t_s, err, dv):
            f.write(f"{t:.6f},{e:.6f},{d:.6f}\n")


def save_plot(path: str, t_s, err, dv):
    plt.figure()
    plt.plot(t_s, err, label="line_err (centroid error)")
    plt.grid(True)
    plt.xlabel("Time [s]")
    plt.ylabel("line_err [QTR units]")
    plt.title("Line Following Performance (line_err vs time)")
    plt.legend(loc="best")
    plt.savefig(path, dpi=200, bbox_inches="tight")
    plt.close()

    # Optional second plot for dv_out
    path2 = path.replace(".png", "_dv.png")
    plt.figure()
    plt.plot(t_s, dv, label="dv_out (steering correction)")
    plt.grid(True)
    plt.xlabel("Time [s]")
    plt.ylabel("dv_out [counts/s]")
    plt.title("Line Following Control Effort (dv_out vs time)")
    plt.legend(loc="best")
    plt.savefig(path2, dpi=200, bbox_inches="tight")
    plt.close()
    return path2


# ------------------------- Main -------------------------

def main():
    ensure_log_dir()
    tag = timestamp()

    csv_path = os.path.join(LOG_DIR, f"{tag}_line_follow.csv")
    png_path = os.path.join(LOG_DIR, f"{tag}_line_err.png")
    raw_path = os.path.join(LOG_DIR, f"{tag}_raw.txt")

    with serial.Serial(PORT, BAUD, timeout=TIMEOUT_S) as ser:
        time.sleep(2.5)
        drain(ser, 1.0)

        sync_to_tuning_ui(ser)

        if SET_LINE_GAINS:
            set_line_gains(ser, KPLINE, KILINE)

        if DO_CALIBRATION:
            calibration_wizard(ser)

        if ENABLE_FOLLOW:
            ensure_follow_enabled(ser)

        # Logging loop
        dt = 1.0 / max(1.0, float(SAMPLE_HZ))
        t0 = time.time()

        times = []
        errs = []
        dvs = []
        raw_blocks = []

        print(f"Logging for {DURATION_S:.2f} s at ~{SAMPLE_HZ:.1f} Hz ...")
        while True:
            t_now = time.time()
            t_rel = t_now - t0
            if t_rel > DURATION_S:
                break

            # Press 'p' and parse
            drain(ser, 0.01)
            send_key(ser, "p")
            ok, txt = read_until_any(ser, needles=[">:"], max_wait_s=3.0)
            raw_blocks.append(txt)

            s = parse_line_status(txt)

            # If missing fields, fill with nan so plots still work
            err = s.get("line_err", float("nan"))
            dv  = s.get("dv_out", float("nan"))

            times.append(t_rel)
            errs.append(err)
            dvs.append(dv)

            time.sleep(dt)

    # Save raw text
    with open(raw_path, "w", encoding="utf-8") as f:
        f.write("\n\n---CAPTURE---\n\n".join(raw_blocks))

    # Save CSV + plots
    save_csv(csv_path, times, errs, dvs)
    dv_png = save_plot(png_path, times, errs, dvs)

    print(f"Saved: {csv_path}")
    print(f"Saved: {png_path}")
    print(f"Saved: {dv_png}")
    print(f"Saved raw log: {raw_path}")

    # Quick sanity print
    arr = np.array(errs, dtype=float)
    finite = np.isfinite(arr)
    if finite.any():
        print(f"line_err: mean={np.mean(arr[finite]):.2f}, std={np.std(arr[finite]):.2f}, min={np.min(arr[finite]):.2f}, max={np.max(arr[finite]):.2f}")
    else:
        print("WARNING: No parsable line_err values. Check raw log + UI output format.")

if __name__ == "__main__":
    main()