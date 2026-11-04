#!/usr/bin/env python3
"""
LineFollower Pure Numbers Controller & Live Tuner
=================================================
Communicates with the LineFollower firmware over HC-05 Bluetooth or Serial
using pure space-separated numbers (<kp> <kd> <baseSpeed> <searchSpeed> <run>).
"""

import argparse
import glob
import json
import os
import sys
import threading
import time
from typing import Callable, Dict, List, Optional

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit(
        "Error: 'pyserial' is required. Please install it with:\n"
        "    pip install pyserial"
    )

try:
    from rich.console import Console
    from rich.panel import Panel
    from rich.table import Table

    console = Console()
    HAS_RICH = True
except ImportError:
    HAS_RICH = False
    console = None


class LineFollowerController:
    """Pure numeric serial interface to LineFollower firmware."""

    def __init__(self, port: str, baud: int = 9600, timeout: float = 0.5):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None
        self._running_listener = False
        self._listener_thread: Optional[threading.Thread] = None
        self._write_lock = threading.Lock()
        self.on_message: Optional[Callable[[str], None]] = None

        # State (pure integers)
        self.state: Dict[str, any] = {
            "running": False,
            "kp": 105,
            "kd": 25,
            "base_speed": 40,
            "search_speed": 80,
        }

    def connect(self) -> None:
        """Open serial port and start background reader."""
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
            time.sleep(0.2)
            self.ser.reset_input_buffer()
        except serial.SerialException as e:
            raise ConnectionError(f"Could not open port '{self.port}': {e}")

        self._running_listener = True
        self._listener_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._listener_thread.start()

    def disconnect(self) -> None:
        """Close connection."""
        self._running_listener = False
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass

    def send_raw(self, line: str) -> None:
        """Send a numeric payload to the robot."""
        if not self.ser or not self.ser.is_open:
            raise ConnectionError("Serial port is not connected.")
        payload = (line.strip() + "\n").encode("utf-8")
        with self._write_lock:
            self.ser.write(payload)
            self.ser.flush()

    def send_state(self) -> None:
        """Send all parameters as space-separated numbers: <kp> <kd> <base> <search> <run>."""
        run_val = 1 if self.state["running"] else 0
        cmd = f"{self.state['kp']} {self.state['kd']} {self.state['base_speed']} {self.state['search_speed']} {run_val}"
        self.send_raw(cmd)

    def _reader_loop(self) -> None:
        """Reads space-separated numbers: <kp> <kd> <base> <search> <run>."""
        while self._running_listener and self.ser and self.ser.is_open:
            try:
                line_bytes = self.ser.readline()
                if not line_bytes:
                    continue
                line = line_bytes.decode("utf-8", errors="replace").strip()
                if line:
                    self._parse_response(line)
                    if self.on_message:
                        self.on_message(line)
            except (serial.SerialException, OSError):
                break

    def _parse_response(self, line: str) -> None:
        """Parse incoming numeric responses (e.g., '105 25 40 80 0')."""
        parts = line.split()
        if len(parts) >= 5:
            try:
                self.state["kp"] = int(parts[0])
                self.state["kd"] = int(parts[1])
                self.state["base_speed"] = int(parts[2])
                self.state["search_speed"] = int(parts[3])
                self.state["running"] = (int(parts[4]) != 0)
            except ValueError:
                pass
        elif len(parts) == 1:
            try:
                val = int(parts[0])
                if val in (0, 1):
                    self.state["running"] = (val == 1)
            except ValueError:
                pass

    # High-level actions
    def set_kp(self, kp: int) -> None:
        self.state["kp"] = int(kp)
        self.send_state()

    def set_kd(self, kd: int) -> None:
        self.state["kd"] = int(kd)
        self.send_state()

    def set_base_speed(self, speed: int) -> None:
        self.state["base_speed"] = max(0, min(255, int(speed)))
        self.send_state()

    def set_search_speed(self, speed: int) -> None:
        self.state["search_speed"] = max(0, min(255, int(speed)))
        self.send_state()

    def start(self) -> None:
        self.state["running"] = True
        self.send_raw("1")

    def stop(self) -> None:
        self.state["running"] = False
        self.send_raw("0")

    def toggle(self) -> None:
        self.state["running"] = not self.state["running"]
        self.send_raw("1" if self.state["running"] else "0")

    def request_status(self) -> None:
        self.send_raw("")

    def step_kp(self, delta: int) -> None:
        self.set_kp(max(0, self.state["kp"] + int(delta)))

    def step_kd(self, delta: int) -> None:
        self.set_kd(max(0, self.state["kd"] + int(delta)))

    def step_base_speed(self, delta: int) -> None:
        self.set_base_speed(max(0, min(255, self.state["base_speed"] + int(delta))))

    def step_search_speed(self, delta: int) -> None:
        self.set_search_speed(max(0, min(255, self.state["search_speed"] + int(delta))))

    # Profile Storage
    def save_profile(self, filepath: str) -> None:
        if not filepath.endswith(".json"):
            filepath += ".json"
        data = {
            "kp": int(self.state["kp"]),
            "kd": int(self.state["kd"]),
            "base_speed": int(self.state["base_speed"]),
            "search_speed": int(self.state["search_speed"]),
            "saved_at": time.strftime("%Y-%m-%d %H:%M:%S")
        }
        with open(filepath, "w") as f:
            json.dump(data, f, indent=4)

    def load_profile(self, filepath: str) -> None:
        if not filepath.endswith(".json"):
            filepath += ".json"
        if not os.path.isfile(filepath):
            raise FileNotFoundError(f"Profile '{filepath}' not found.")
        with open(filepath, "r") as f:
            data = json.load(f)

        if "kp" in data:
            self.state["kp"] = int(data["kp"])
        if "kd" in data:
            self.state["kd"] = int(data["kd"])
        if "base_speed" in data:
            self.state["base_speed"] = int(data["base_speed"])
        if "search_speed" in data:
            self.state["search_speed"] = int(data["search_speed"])
        self.send_state()


def find_serial_ports() -> List[str]:
    """Finds available RFCOMM and USB/Serial ports."""
    ports = serial.tools.list_ports.comports()
    matched = []

    rfcomm_devices = glob.glob("/dev/rfcomm*")
    matched.extend(rfcomm_devices)

    for p in ports:
        name = p.device
        desc = p.description.lower()
        if any(term in name.lower() or term in desc for term in ["rfcomm", "bluetooth", "usb", "acm", "ch340", "cp210", "ftdi"]):
            if name not in matched:
                matched.append(name)
        elif not name.startswith("/dev/ttyS") and name not in matched:
            matched.append(name)

    return matched if matched else [p.device for p in ports]


def print_ui_table(state: Dict[str, any], steps: Dict[str, any]) -> None:
    """Renders current parameters table."""
    status_str = "RUNNING" if state["running"] else "STOPPED"
    if HAS_RICH:
        table = Table(title=f"LineFollower Status [{status_str}]", title_style="bold magenta")
        table.add_column("Parameter", style="cyan", no_wrap=True)
        table.add_column("Value (int)", style="bold white")
        table.add_column("Step (int)", style="yellow")
        table.add_column("Quick Tune Commands", style="green")

        table.add_row("State", f"[bold green]{status_str}[/]" if state["running"] else f"[bold red]{status_str}[/]", "-", "start (1) / stop (0)")
        table.add_row("Kp (Proportional)", f"{state['kp']}", f"±{steps['kp']}", "p <val> | +p | -p")
        table.add_row("Kd (Derivative)", f"{state['kd']}", f"±{steps['kd']}", "d <val> | +d | -d")
        table.add_row("Base Speed", f"{state['base_speed']}", f"±{steps['base_speed']}", "b <val> | +b | -b")
        table.add_row("Search Speed", f"{state['search_speed']}", f"±{steps['search_speed']}", "s <val> | +s | -s")
        console.print(table)
    else:
        print("\n" + "=" * 50)
        print(f" LineFollower Status: [{status_str}]")
        print("=" * 50)
        print(f"  Kp (Proportional): {state['kp']:4d} (step ±{steps['kp']})  | 'p <val>', '+p', '-p'")
        print(f"  Kd (Derivative)  : {state['kd']:4d} (step ±{steps['kd']})  | 'd <val>', '+d', '-d'")
        print(f"  Base Speed       : {state['base_speed']:4d} (step ±{steps['base_speed']})  | 'b <val>', '+b', '-b'")
        print(f"  Search Speed     : {state['search_speed']:4d} (step ±{steps['search_speed']})  | 's <val>', '+s', '-s'")
        print("=" * 50 + "\n")


def interactive_session(controller: LineFollowerController) -> None:
    """Interactive command shell with numeric communication."""
    step_sizes = {
        "kp": 5,
        "kd": 2,
        "base_speed": 5,
        "search_speed": 5,
    }

    def on_robot_line(line: str):
        if HAS_RICH:
            console.print(f"[bold cyan]Robot Numbers:[/] {line}")
        else:
            print(f">> {line}")

    controller.on_message = on_robot_line

    if HAS_RICH:
        console.print(Panel.fit(
            f"[bold green]Connected to LineFollower on {controller.port}[/]\n"
            f"[dim]Numbers Protocol: '<kp> <kd> <base> <search> <run>'\n"
            f"Commands: 'p 120', 'd 30', 'b 50', '1' (start), '0' (stop), '+p'/'-p', 'q' to quit.[/dim]",
            title="LineFollower Numbers Tuner",
            border_style="cyan"
        ))
    else:
        print(f"\n--- Connected to {controller.port} (9600 baud) ---")
        print("Commands: 'p 120', 'd 30', 'b 50', '1' (start), '0' (stop), '+p'/'-p', 'q' to quit.\n")

    controller.request_status()
    time.sleep(0.2)
    print_ui_table(controller.state, step_sizes)

    while True:
        try:
            status_tag = "RUN" if controller.state["running"] else "STOP"
            prompt_str = f"LineFollower [{status_tag} | Kp={controller.state['kp']} Kd={controller.state['kd']} Base={controller.state['base_speed']}] > "
            user_input = input(prompt_str).strip()
            if not user_input:
                continue

            parts = user_input.split()
            cmd = parts[0].lower()
            arg = parts[1] if len(parts) > 1 else ""

            # Exit
            if cmd in ("exit", "quit", "q"):
                if controller.state["running"]:
                    controller.stop()
                    print("Sent stop (0) to robot.")
                print("Exiting controller.")
                break

            # Help
            elif cmd in ("help", "h"):
                print("""
Numeric Commands:
  <kp> <kd>        : Send numbers directly (e.g. '120 30' or '120 30 50')
  1 / start        : Start robot (sends '1')
  0 / stop / s     : Stop robot (sends '0')
  p <val>          : Set Kp integer
  d <val>          : Set Kd integer
  b <val>          : Set Base Speed integer (0-255)
  s <val>          : Set Search Speed integer (0-255)
  +p / -p          : Bump Kp by step size
  +d / -d          : Bump Kd by step size
  +b / -b          : Bump Base Speed by step size
  +s / -s          : Bump Search Speed by step size
  step <param> <v> : Set step size (e.g. 'step p 10')
  status / ?       : Display parameters table
  save <name>      : Save profile to <name>.json
  load <name>      : Load profile from <name>.json
  q / exit         : Exit
""")

            # Direct numeric input (e.g. '120 30' or '1' or '0')
            elif all(p.lstrip("-").isdigit() for p in parts):
                controller.send_raw(user_input)

            # Status / Table
            elif cmd in ("status", "?", "table"):
                controller.request_status()
                time.sleep(0.15)
                print_ui_table(controller.state, step_sizes)

            # Start / Stop / Toggle
            elif cmd in ("start", "run"):
                controller.start()
            elif cmd in ("stop", "halt"):
                controller.stop()
            elif cmd in ("toggle", "t"):
                controller.toggle()

            # Increments / Decrements
            elif cmd == "+p":
                controller.step_kp(step_sizes["kp"])
            elif cmd == "-p":
                controller.step_kp(-step_sizes["kp"])
            elif cmd == "+d":
                controller.step_kd(step_sizes["kd"])
            elif cmd == "-d":
                controller.step_kd(-step_sizes["kd"])
            elif cmd == "+b":
                controller.step_base_speed(step_sizes["base_speed"])
            elif cmd == "-b":
                controller.step_base_speed(-step_sizes["base_speed"])
            elif cmd == "+s":
                controller.step_search_speed(step_sizes["search_speed"])
            elif cmd == "-s":
                controller.step_search_speed(-step_sizes["search_speed"])

            # Setters
            elif cmd in ("p", "kp") and arg:
                controller.set_kp(int(arg))
            elif cmd in ("d", "kd") and arg:
                controller.set_kd(int(arg))
            elif cmd in ("b", "base") and arg:
                controller.set_base_speed(int(arg))
            elif cmd in ("s", "search") and arg:
                controller.set_search_speed(int(arg))

            # Configure Step Size
            elif cmd == "step":
                if len(parts) >= 3:
                    target_param = parts[1].lower()
                    val = int(parts[2])
                    if target_param in ("p", "kp"):
                        step_sizes["kp"] = val
                    elif target_param in ("d", "kd"):
                        step_sizes["kd"] = val
                    elif target_param in ("b", "base"):
                        step_sizes["base_speed"] = val
                    elif target_param in ("s", "search"):
                        step_sizes["search_speed"] = val
                    print(f"Step size for '{target_param}' set to {val}.")
                else:
                    print("Usage: step <param> <val> (e.g. 'step p 10')")

            # Presets
            elif cmd == "save":
                filename = arg if arg else "profile"
                controller.save_profile(filename)
                print(f"Profile saved to '{filename}.json'")
            elif cmd == "load":
                filename = arg if arg else "profile"
                try:
                    controller.load_profile(filename)
                    print(f"Profile loaded from '{filename}.json'")
                except FileNotFoundError as err:
                    print(f"Error: {err}")

            else:
                controller.send_raw(user_input)

            time.sleep(0.05)

        except KeyboardInterrupt:
            print("\nInterrupt detected. Stopping robot...")
            controller.stop()
            break
        except Exception as err:
            print(f"Error: {err}")


def main():
    parser = argparse.ArgumentParser(
        description="LineFollower Pure Numbers Controller via HC-05 BLE / USB Serial"
    )
    parser.add_argument("-p", "--port", type=str, help="Serial port (e.g., /dev/rfcomm0, COM3, /dev/ttyUSB0)")
    parser.add_argument("-b", "--baud", type=int, default=9600, help="Baud rate (default: 9600)")
    parser.add_argument("--kp", type=int, help="Set Kp integer parameter")
    parser.add_argument("--kd", type=int, help="Set Kd integer parameter")
    parser.add_argument("--base", type=int, help="Set base motor speed integer (0-255)")
    parser.add_argument("--search", type=int, help="Set search recovery speed integer (0-255)")
    parser.add_argument("--start", action="store_true", help="Start the robot (1)")
    parser.add_argument("--stop", action="store_true", help="Stop the robot (0)")
    parser.add_argument("--toggle", action="store_true", help="Toggle run/stop state")
    parser.add_argument("--status", action="store_true", help="Query and print status")
    parser.add_argument("--load", type=str, help="Load a preset profile JSON file")
    parser.add_argument("--save", type=str, help="Save current profile to JSON file")
    parser.add_argument("-i", "--interactive", action="store_true", help="Force interactive REPL session")

    args = parser.parse_args()

    port = args.port
    if not port:
        available_ports = find_serial_ports()
        if not available_ports:
            sys.exit(
                "No serial or Bluetooth RFCOMM ports detected.\n"
                "Please specify a port with --port /dev/rfcomm0 (or COMx on Windows)."
            )
        elif len(available_ports) == 1:
            port = available_ports[0]
            print(f"Auto-selected detected port: {port}")
        else:
            print("Multiple serial ports found:")
            for idx, p in enumerate(available_ports, 1):
                print(f"  [{idx}] {p}")
            try:
                choice = input(f"Select port [1-{len(available_ports)}] (default: 1): ").strip()
                selected_idx = int(choice) - 1 if choice else 0
                port = available_ports[selected_idx]
            except (ValueError, IndexError):
                port = available_ports[0]
            print(f"Selected port: {port}")

    controller = LineFollowerController(port, baud=args.baud)
    try:
        controller.connect()
    except ConnectionError as e:
        sys.exit(f"Connection failed: {e}")

    try:
        has_direct_action = any([
            args.kp is not None,
            args.kd is not None,
            args.base is not None,
            args.search is not None,
            args.start,
            args.stop,
            args.toggle,
            args.status,
            args.load is not None,
            args.save is not None,
        ])

        if has_direct_action and not args.interactive:
            time.sleep(0.15)
            if args.load:
                controller.load_profile(args.load)
                print(f"Loaded profile: {args.load}")
            if args.kp is not None:
                controller.state["kp"] = args.kp
            if args.kd is not None:
                controller.state["kd"] = args.kd
            if args.base is not None:
                controller.state["base_speed"] = args.base
            if args.search is not None:
                controller.state["search_speed"] = args.search
            if args.start:
                controller.state["running"] = True
            elif args.stop:
                controller.state["running"] = False
            elif args.toggle:
                controller.state["running"] = not controller.state["running"]

            controller.send_state()

            if args.save:
                time.sleep(0.2)
                controller.save_profile(args.save)
                print(f"Saved profile to: {args.save}")
            time.sleep(0.3)
        else:
            interactive_session(controller)

    finally:
        controller.disconnect()


if __name__ == "__main__":
    main()
