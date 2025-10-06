#!/usr/bin/env python3
import csv
import sys
import time
import can

# --- Hard-coded config ---
CSV_FILE   = "00000001_CAN.csv"
INTERFACE  = "slcan"
CHANNEL    = "COM10"       # Windows COM port
CAN_BITRATE = 500000       # CAN bus bitrate
SLEEP_BETWEEN_S = 0.001    # fixed 5 ms between frames

def load_csv(path):
    """
    Read semicolon-delimited CSV with columns including:
    TimestampEpoch;BusChannel;ID;IDE;...;DataBytes
    We only use ID, IDE, DataBytes.
    """
    records = []
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f, delimiter=";")
        need = {"ID", "IDE", "DataBytes"}
        if not reader.fieldnames or not need.issubset(set(reader.fieldnames)):
            raise ValueError(f"CSV missing required columns {need}")

        for row in reader:
            try:
                # Treat ID as HEX (typical for CAN logs)
                arb_id = int(row["ID"].strip(), 16)
            except Exception:
                continue

            is_extended = row["IDE"].strip() == "1"

            data_hex = (row.get("DataBytes") or "").strip()
            if len(data_hex) % 2 != 0:
                data_hex = "0" + data_hex
            try:
                data = bytes.fromhex(data_hex) if data_hex else b""
            except ValueError:
                continue

            # python-can uses len(data) as DLC for classical CAN
            if len(data) > 8:
                data = data[:8]  # clamp to classical CAN
            records.append((arb_id, is_extended, data))

    if not records:
        raise ValueError("No valid frames parsed from CSV.")
    return records


def open_bus():
    # Try with explicit slcan serial baud first (some adapters need it),
    # fall back to default if unsupported.
    try:
        return can.Bus(interface=INTERFACE, channel=CHANNEL, bitrate=CAN_BITRATE)
    except Exception:
        # Fallback attempt without extra args failed already; re-raise original
        raise


def replay_fixed_5ms(records, bus):
    for i, (arb_id, is_ext, data) in enumerate(records):
        try:
            msg = can.Message(arbitration_id=arb_id, is_extended_id=is_ext, data=data, is_fd=False)
            bus.send(msg)
        except can.CanError as e:
            print(f"Send error at index {i} ID=0x{arb_id:X}: {e}", file=sys.stderr)
        # Sleep 5 ms between messages (skip after last one)
        if i + 1 < len(records):
            time.sleep(SLEEP_BETWEEN_S)


def main():
    try:
        recs = load_csv(CSV_FILE)
    except Exception as e:
        print(f"Failed to load CSV: {e}", file=sys.stderr)
        sys.exit(1)

    try:
        bus = open_bus()
    except Exception as e:
        print(f"Failed to open CAN bus ({INTERFACE}:{CHANNEL}): {e}", file=sys.stderr)
        sys.exit(2)

    print(f"Replaying {len(recs)} frames from {CSV_FILE} on {INTERFACE}:{CHANNEL} @ {CAN_BITRATE} bit/s")
    try:
        replay_fixed_5ms(recs, bus)
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        try:
            bus.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
