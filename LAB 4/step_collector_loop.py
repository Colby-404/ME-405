"""
step_collector_loop.py
Automates step response collection by talking to the Nucleo UI over serial.

MM/S VERSION:
- Assumes the MCU prints and logs: Time [us], Speed [mm/s]
- Assumes setpoint share is in mm/s
"""

import os
import time
from datetime import datetime

import serial
import matplotlib.pyplot as plt
import numpy as np

# ------------------------- User Settings -------------------------
PORT = "COM9"
BAUD = 115200
TIMEOUT_S = 0.15

LOG_DIR = "Collection Log"
DEBUG = True

TIME_INPUT_UNITS = "us"   # "us" if MCU prints ticks_us
PROMPTS = (">: ", ">:")   # UI prompt variants

# ---- Signal from MCU ----
# In your updated firmware, the second column is Speed [mm/s]
DATA_KIND = "speed_mm_s"
Y_NAME = "Speed"
Y_UNITS_LABEL = "mm/s"
TARGET_LINE_VALUE_FROM_META = True   # setpoint is speed (mm/s)
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


def read_some(ser: serial.Serial, n=256) -> bytes:
    return ser.read(n)


def drain(ser: serial.Serial, seconds=0.4) -> None:
    t0 = time.time()
    while time.time() - t0 < seconds:
        chunk = read_some(ser, 512)
        if not chunk:
            time.sleep(0.01)


def _text_has_any_prompt(text_lower: str) -> bool:
    return any(p.lower() in text_lower for p in PROMPTS)


def read_until_any(ser: serial.Serial, needles, max_wait_s=8.0):
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
                _dbg("RX_TAIL:", printable[-240:])

            low = text.lower()
            if all(n in low for n in needles_low):
                return True, text
        else:
            time.sleep(0.01)

    return False, buf.decode("utf-8", errors="replace")


def sync_to_tuning_ui(ser: serial.Serial, max_wait_s=12.0) -> str:
    """Force tuning UI to show menu/prompt."""
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

def parse_csv_from_text(text: str):
    """
    Parse ONLY the data table printed by task_user:
      Time [us], Speed [mm/s]
      <t>,<y>
      <t>,<y>
      ...
    Stops parsing at footer ("Commands:" or dashed line).
    """
    t_list, y_list = [], []
    in_table = False

    for line in text.splitlines():
        low = line.lower()

        # Stop if footer begins
        if in_table and ("commands:" in low or "----" in low):
            break

        # Start parsing only after the header line
        if ("time [us]" in low) and ("," in line):
            in_table = True
            continue

        if not in_table:
            continue

        parts = [p.strip() for p in line.split(",")]
        if len(parts) != 2:
            continue

        try:
            t = float(parts[0])
            y = float(parts[1])
        except ValueError:
            continue

        t_list.append(t)
        y_list.append(y)

    return t_list, y_list


def convert_time_to_seconds(t: float) -> float:
    return t * 1e-6 if TIME_INPUT_UNITS == "us" else t


# ------------------------- UI Actions -------------------------

def set_gains_and_setpoint(ser, kp, ki, setpoint_mm_s):
    sync_to_tuning_ui(ser)
    drain(ser, 0.2)

    send_key(ser, "k")
    ok, _ = read_until_any(ser, needles=["Enter proportional gain"], max_wait_s=8.0)
    if not ok:
        raise RuntimeError("Timed out waiting for Kp prompt.")
    send_number(ser, kp)

    ok, _ = read_until_any(ser, needles=["Enter integral gain"], max_wait_s=8.0)
    if not ok:
        raise RuntimeError("Timed out waiting for Ki prompt.")
    send_number(ser, ki)

    ok, _ = read_until_any(ser, needles=[">:"], max_wait_s=6.0)
    if not ok:
        raise RuntimeError("Timed out returning to tuning prompt after Ki.")

    send_key(ser, "s")

    # Your updated firmware prompt SHOULD be "Enter setpoint (mm/s)"
    # Keep fallback for older firmware ("Enter setpoint value")
    ok, _ = read_until_any(ser, needles=["Enter setpoint (mm/s)"], max_wait_s=6.0)
    if not ok:
        ok, _ = read_until_any(ser, needles=["Enter setpoint value"], max_wait_s=6.0)
        if not ok:
            raise RuntimeError("Timed out waiting for setpoint prompt.")

    send_number(ser, setpoint_mm_s)

    ok, _ = read_until_any(ser, needles=[">:"], max_wait_s=6.0)
    if not ok:
        raise RuntimeError("Timed out returning to tuning prompt after setpoint.")


def collect_step_response_single_side(ser, side="l"):
    """
    Runs ONE test in data UI for side 'l' or 'r' and returns parsed data.
    Assumes gains/setpoint already set.
    """
    raw_parts = []

    # Enter data UI
    send_key(ser, "g")
    ok, text = read_until_any(ser, needles=["User task active"], max_wait_s=10.0)
    raw_parts.append(text)
    if not ok:
        raise RuntimeError("Timed out entering data UI.")

    side = (side or "l").lower().strip()
    if side not in ("l", "r"):
        side = "l"
    send_key(ser, side)

    ok, text = read_until_any(ser, needles=["Data collection complete"], max_wait_s=20.0)
    raw_parts.append(text)
    if not ok:
        raise RuntimeError("Timed out waiting for 'Data collection complete'.")

    ok, text = read_until_any(ser, needles=["Commands:", ">:"], max_wait_s=25.0)
    raw_parts.append(text)
    if not ok:
        raise RuntimeError("Timed out waiting for data UI to return prompt.")

    # Return to tuning menu so we can run the other side cleanly
    send_key(ser, "h")
    ok, text = read_until_any(ser, needles=["ME 405 Romi Tuning Interface"], max_wait_s=10.0)
    raw_parts.append(text)

    raw_text = "\n\n---CAPTURE---\n\n".join(raw_parts)
    t_raw, y_raw = parse_csv_from_text(raw_text)
    return t_raw, y_raw, raw_text


# ------------------------- Output -------------------------

def save_csv(path: str, t_s, y, meta: dict, label: str):
    with open(path, "w", encoding="utf-8") as f:
        f.write(f"# label: {label}\n")
        f.write(f"# data_kind: {DATA_KIND}\n")
        f.write(f"# setpoint_mm_s: {meta['setpoint_mm_s']}\n")
        f.write(f"# kp: {meta['kp']}\n")
        f.write(f"# ki: {meta['ki']}\n")
        f.write(f"Time [s],{Y_NAME} [{Y_UNITS_LABEL}]\n")
        for t, yy in zip(t_s, y):
            f.write(f"{t:.6f},{yy:.9f}\n")


def save_plot(path: str, tL, yL, tR, yR, meta):
    plt.figure()

    plt.plot(tL, yL, label="Left Motor")
    plt.plot(tR, yR, label="Right Motor")

    plt.xlim(0, 3)
    plt.grid(True)
    plt.xlabel("Time [s]")
    plt.ylabel(f"{Y_NAME} [{Y_UNITS_LABEL}]")
    plt.title(f"Step Response | sp={meta['setpoint_mm_s']} mm/s  kp={meta['kp']}  ki={meta['ki']}")

    if TARGET_LINE_VALUE_FROM_META:
        plt.axhline(meta["setpoint_mm_s"], linestyle="--", label="Setpoint")

    plt.legend(loc="best")
    plt.savefig(path, dpi=200, bbox_inches="tight")
    plt.close()


def compute_step_metrics(t_s, y, meta: dict):
    """
    Returns:
      y_ss, y_pk, t_pk, os_setpoint_percent
    """
    y = np.array(y, dtype=float)
    t = np.array(t_s, dtype=float)
    if len(y) < 2:
        return None

    N = max(20, len(y) // 10)
    N = min(N, len(y))
    y_ss = float(np.mean(y[-N:]))
    y_pk = float(np.max(y))
    t_pk = float(t[int(np.argmax(y))])

    os_setpoint = float("nan")
    sp = float(meta["setpoint_mm_s"])
    if TARGET_LINE_VALUE_FROM_META and abs(sp) > 1e-12:
        os_setpoint = (y_pk - sp) / sp * 100.0

    return y_ss, y_pk, t_pk, os_setpoint


# ------------------------- Main -------------------------

def main():
    ensure_log_dir()
    tag = timestamp()

    # ---- choose test (MM/S) ----
    setpoint_mm_s = 300.0
    kp = 0.15
    ki = 0.01
    # ---------------------------

    meta = {"setpoint_mm_s": float(setpoint_mm_s), "kp": float(kp), "ki": float(ki)}

    csv_left_path  = os.path.join(LOG_DIR, f"{tag}_left.csv")
    csv_right_path = os.path.join(LOG_DIR, f"{tag}_right.csv")
    png_path = os.path.join(LOG_DIR, f"{tag}.png")
    raw_path = os.path.join(LOG_DIR, f"{tag}_raw.txt")

    with serial.Serial(PORT, BAUD, timeout=TIMEOUT_S) as ser:
        time.sleep(2.5)
        drain(ser, 1.0)

        # 1) Set gains/setpoint once
        set_gains_and_setpoint(ser, kp, ki, setpoint_mm_s)

        # 2) Collect LEFT then RIGHT (no mixing, clean legend)
        t_raw_L, y_raw_L, raw_L = collect_step_response_single_side(ser, side="l")
        t_raw_R, y_raw_R, raw_R = collect_step_response_single_side(ser, side="r")

    # Convert times
    tL = [convert_time_to_seconds(t) for t in t_raw_L]
    yL = list(y_raw_L)

    tR = [convert_time_to_seconds(t) for t in t_raw_R]
    yR = list(y_raw_R)

    # Save raw combined
    with open(raw_path, "w", encoding="utf-8") as f:
        f.write("===== LEFT RAW =====\n")
        f.write(raw_L)
        f.write("\n\n===== RIGHT RAW =====\n")
        f.write(raw_R)

    if len(tL) < 5 or len(tR) < 5:
        print("WARNING: Very little data parsed. Check raw log:", raw_path)

    # Save CSVs
    save_csv(csv_left_path, tL, yL, meta, label="LEFT")
    save_csv(csv_right_path, tR, yR, meta, label="RIGHT")

    # Save plot (both with legend)
    if len(tL) >= 2 and len(tR) >= 2:
        save_plot(png_path, tL, yL, tR, yR, meta)

    print(f"Saved: {csv_left_path}")
    print(f"Saved: {csv_right_path}")
    print(f"Saved: {png_path}")
    print(f"Saved raw log: {raw_path}")

    # Metrics per side
    mL = compute_step_metrics(tL, yL, meta)
    mR = compute_step_metrics(tR, yR, meta)

    if mL:
        y_ss, y_pk, t_pk, os_sp = mL
        print("\nLEFT METRICS")
        print(f"y_ss = {y_ss:.6f} [{Y_UNITS_LABEL}]")
        print(f"y_pk = {y_pk:.6f} [{Y_UNITS_LABEL}] at t = {t_pk:.4f} s")
        if not np.isnan(os_sp):
            print(f"%OS (relative to setpoint) = {os_sp:.2f}%")

    if mR:
        y_ss, y_pk, t_pk, os_sp = mR
        print("\nRIGHT METRICS")
        print(f"y_ss = {y_ss:.6f} [{Y_UNITS_LABEL}]")
        print(f"y_pk = {y_pk:.6f} [{Y_UNITS_LABEL}] at t = {t_pk:.4f} s")
        if not np.isnan(os_sp):
            print(f"%OS (relative to setpoint) = {os_sp:.2f}%")


if __name__ == "__main__":
    main()