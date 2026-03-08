from matplotlib import pyplot as plt
from pathlib import Path
import sys

# ------------------------------------------------------------
# Motor data plotter (CSV-cleaning + rejection reporting)
# Expected file format:
#   Time,Omega
#   12345,6.52
#   ...
# Lines can include:
#   - empty lines
#   - full-line comments starting with '#'
#   - inline comments after a '#'
# ------------------------------------------------------------

#look for motor_dataL.csv
SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CSV = SCRIPT_DIR / "motor_dataL.csv"
csv_path = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_CSV

if not csv_path.exists():
    print("ERROR: CSV file not found.")
    print("Tried:", csv_path)
    print("Script folder:", SCRIPT_DIR)
    print("Current working directory:", Path.cwd())
    print("\nFiles in script folder:")

data_t = []   # time (s)
data_w = []   # omega

with open("motor_dataL.csv", "r") as file:

    # Read header (line 1)
    header = file.readline().strip()

    # Loop remaining lines (starting at line 2)
    for line_num, line in enumerate(file, start=2):
        raw = line  # keep original for debugging if you want
        line = line.strip()

        # Reject: empty line
        if not line:
            print(f"Rejected line {line_num}: Empty line")
            continue

        # Reject: full-line comment
        if line.startswith("#"):
            print(f"Rejected line {line_num}: Comment line")
            continue

        # Remove inline comments (anything after '#')
        if "#" in line:
            line = line.split("#", 1)[0].strip()

        # Reject: line became empty after stripping inline comment
        if not line:
            print(f"Rejected line {line_num}: Only inline comment")
            continue

        # Split on comma
        parts = [p.strip().strip('"') for p in line.split(",") if p.strip() != ""]

        # Reject: not enough columns
        if len(parts) > 2:
            # Keep only the first two columns (Time, Omega) and ignore the rest
            parts = parts[:2]


        # Reject: too many columns
        # If your print has a trailing comma, len(parts) will still be 2 because
        # we removed empty strings above. So this is safe.
        if len(parts) > 2:
            print(f"Rejected line {line_num}: Too many columns")
            continue

        # Parse numbers
        try:
            t_us = float(parts[0])
            omega = float(parts[1])
        except ValueError:
            print(f"Rejected line {line_num}: Non-numeric data -> {parts}")
            continue

        # Convert microseconds to seconds
        t_s = t_us / 1e6

        data_t.append(t_s)
        data_w.append(omega)
        
print("Header:", header if header else "(none)")
print("Points read:", len(data_t))
print("First 5 points:", list(zip(data_t, data_w))[:5])

# Basic sanity check
if len(data_t) == 0:
    raise RuntimeError("No valid data rows found. Check your data.csv formatting.")

# Plot
plt.plot(data_t, data_w)
plt.xlabel("Time (s)")
plt.ylabel("Omega")
plt.title(header if header else "Motor Data")
plt.grid(True)
plt.show()