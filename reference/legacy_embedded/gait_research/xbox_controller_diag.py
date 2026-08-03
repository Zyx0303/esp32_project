import ctypes
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parent
VENDORED_INPUTS = ROOT / ".deps" / "site"

if VENDORED_INPUTS.exists():
    sys.path.insert(0, str(VENDORED_INPUTS))


def try_import_inputs():
    try:
        import inputs  # type: ignore

        return inputs
    except Exception as exc:
        print(f"inputs import failed: {exc}")
        return None


class XINPUT_GAMEPAD(ctypes.Structure):
    _fields_ = [
        ("wButtons", ctypes.c_ushort),
        ("bLeftTrigger", ctypes.c_ubyte),
        ("bRightTrigger", ctypes.c_ubyte),
        ("sThumbLX", ctypes.c_short),
        ("sThumbLY", ctypes.c_short),
        ("sThumbRX", ctypes.c_short),
        ("sThumbRY", ctypes.c_short),
    ]


class XINPUT_STATE(ctypes.Structure):
    _fields_ = [
        ("dwPacketNumber", ctypes.c_ulong),
        ("Gamepad", XINPUT_GAMEPAD),
    ]


XINPUT_BUTTONS = [
    (0x0001, "DPAD_UP"),
    (0x0002, "DPAD_DOWN"),
    (0x0004, "DPAD_LEFT"),
    (0x0008, "DPAD_RIGHT"),
    (0x0010, "START"),
    (0x0020, "BACK"),
    (0x0040, "L3"),
    (0x0080, "R3"),
    (0x0100, "LB"),
    (0x0200, "RB"),
    (0x1000, "A"),
    (0x2000, "B"),
    (0x4000, "X"),
    (0x8000, "Y"),
]


def load_xinput():
    for dll_name in ("xinput1_4", "xinput9_1_0", "xinput1_3"):
        try:
            dll = ctypes.WinDLL(dll_name)
            get_state = dll.XInputGetState
            get_state.argtypes = [ctypes.c_uint, ctypes.POINTER(XINPUT_STATE)]
            get_state.restype = ctypes.c_uint
            return dll_name, get_state
        except Exception:
            continue
    return None, None


def decode_buttons(mask):
    names = [name for bit, name in XINPUT_BUTTONS if mask & bit]
    return names if names else ["NONE"]


def dump_xinput_slots(samples=5, interval_s=1.0):
    dll_name, get_state = load_xinput()
    if get_state is None:
        print("XInput DLL not available")
        return False

    print(f"XInput DLL: {dll_name}")
    any_connected = False
    for sample_idx in range(samples):
        print(f"sample {sample_idx}")
        for slot in range(4):
            state = XINPUT_STATE()
            ret = get_state(slot, ctypes.byref(state))
            connected = ret == 0
            any_connected = any_connected or connected
            print(
                "slot={slot} ret={ret} packet={packet} buttons=0x{buttons:04x} "
                "LT={lt} RT={rt} LX={lx} LY={ly} RX={rx} RY={ry}".format(
                    slot=slot,
                    ret=ret,
                    packet=state.dwPacketNumber,
                    buttons=state.Gamepad.wButtons,
                    lt=state.Gamepad.bLeftTrigger,
                    rt=state.Gamepad.bRightTrigger,
                    lx=state.Gamepad.sThumbLX,
                    ly=state.Gamepad.sThumbLY,
                    rx=state.Gamepad.sThumbRX,
                    ry=state.Gamepad.sThumbRY,
                )
            )
        time.sleep(interval_s)
    return any_connected


def live_xinput_monitor(duration_s=15.0, interval_s=0.05):
    dll_name, get_state = load_xinput()
    if get_state is None:
        print("XInput DLL not available")
        return False

    print(f"=== XInput live monitor via {dll_name} ===")
    deadline = time.time() + duration_s
    last_packet = {}
    any_connected = False
    any_changed = False

    while time.time() < deadline:
        for slot in range(4):
            state = XINPUT_STATE()
            ret = get_state(slot, ctypes.byref(state))
            if ret != 0:
                continue

            any_connected = True
            packet = state.dwPacketNumber
            if last_packet.get(slot) == packet:
                continue

            any_changed = True
            last_packet[slot] = packet
            print(
                "slot={slot} packet={packet} buttons={buttons} LT={lt} RT={rt} "
                "LX={lx} LY={ly} RX={rx} RY={ry}".format(
                    slot=slot,
                    packet=packet,
                    buttons=",".join(decode_buttons(state.Gamepad.wButtons)),
                    lt=state.Gamepad.bLeftTrigger,
                    rt=state.Gamepad.bRightTrigger,
                    lx=state.Gamepad.sThumbLX,
                    ly=state.Gamepad.sThumbLY,
                    rx=state.Gamepad.sThumbRX,
                    ry=state.Gamepad.sThumbRY,
                )
            )
        time.sleep(interval_s)

    if any_connected and not any_changed:
        print("Controller connected but no state changes were observed")
    if not any_connected:
        print("No connected XInput controller was found")
    return any_connected


def dump_inputs_devices(inputs_mod):
    gamepads = list(inputs_mod.devices.gamepads)
    keyboards = list(inputs_mod.devices.keyboards)
    mice = list(inputs_mod.devices.mice)
    print(f"inputs version: {getattr(inputs_mod, '__version__', 'unknown')}")
    print(f"inputs gamepads: {gamepads}")
    print(f"inputs keyboards: {keyboards}")
    print(f"inputs mice: {mice}")
    return gamepads


def read_gamepad_events(inputs_mod, duration_s=10):
    gamepads = dump_inputs_devices(inputs_mod)
    if not gamepads:
        print("No gamepad found by inputs")
        return

    gamepad = gamepads[0]
    print(f"Reading events from: {gamepad}")
    deadline = time.time() + duration_s
    while time.time() < deadline:
        events = gamepad.read()
        for event in events:
            print(
                f"type={event.ev_type} code={event.code} state={event.state}"
            )


def main():
    print("=== XInput scan ===")
    xinput_connected = dump_xinput_slots()
    if xinput_connected:
        print("Move a stick or press A/B/LB/RB now...")
        live_xinput_monitor()

    print("=== inputs scan ===")
    inputs_mod = try_import_inputs()
    if inputs_mod is None:
        return

    gamepads = dump_inputs_devices(inputs_mod)
    if gamepads:
        print("=== inputs event read ===")
        read_gamepad_events(inputs_mod)
    elif not xinput_connected:
        print("No active controller detected by either XInput or inputs")


if __name__ == "__main__":
    main()
