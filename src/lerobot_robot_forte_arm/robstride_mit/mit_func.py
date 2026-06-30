
# ── Mode bytes (must match foc.h) ────────────────────────────────────────────
MENU_MODE = 0
MIT_MODE  = 5

# ── Scaling ranges — must match flash-stored values on the motor ─────────────
# P_MIN   = -12.5     # rad
# P_MAX   =  12.5     # rad
P_MAX = 12.57
P_MIN = -P_MAX
V_MIN   = -65.0     # rad/s
V_MAX   =  65.0     # rad/s
KP_MAX  =  500.0    # N-m/rad
KD_MAX  =  5.0      # N-m·s/rad
GR = 18.0
KT_AFTER_REDUCER = 2.97
KT = 2.97/GR
I_MAX = 40.0
T_MIN   = -40.0 * GR * KT   # N-m
T_MAX   =  40.0 * GR * KT   # N-m

# ── Sine trajectory parameters ────────────────────────────────────────────────
AMPLITUDE   = 3.14/4.0 # rad   (peak displacement from zero)
FREQUENCY   = 0.2 # Hz
UPDATE_HZ   = 200   # command rate
CENTER      = 0.0   # rad   (center of sine wave)

# ── Control gains ─────────────────────────────────────────────────────────────
KP = 40   # N-m/rad
KD = 1.0    # N-m·s/rad

# ── Plot settings ─────────────────────────────────────────────────────────────
PLOT_WINDOW_S = 5   # seconds of history shown
PLOT_FPS      = 30  # max render rate — independent of UPDATE_HZ


def float_to_uint(x, x_min, x_max, bits):
    x = max(x_min, min(x_max, x))
    return int((x - x_min) / (x_max - x_min) * ((1 << bits) - 1))

def uint_to_float(x, x_min, x_max, bits):
    return x * (x_max - x_min) / ((1 << bits) - 1) + x_min

def pack_cmd(p, v, kp, kd, t_ff):
    p_int  = float_to_uint(p,    P_MIN,  P_MAX,  16)
    v_int  = float_to_uint(v,    V_MIN,  V_MAX,  12)
    kp_int = float_to_uint(kp,   0,      KP_MAX, 12)
    kd_int = float_to_uint(kd,   0,      KD_MAX, 12)
    t_int  = float_to_uint(t_ff, T_MIN,  T_MAX,  12)
    return [
        p_int >> 8,
        p_int & 0xFF,
        v_int >> 4,
        ((v_int & 0xF) << 4) | (kp_int >> 8),
        kp_int & 0xFF,
        kd_int >> 4,
        ((kd_int & 0xF) << 4) | (t_int >> 8),
        t_int & 0xFF,
    ]

def decode_reply(d):
    
    if len(d) < 8:
        return None
    p_int = (d[1] << 8) | d[2]
    v_int = (d[3] << 4) | (d[4] >> 4)
    t_int = ((d[4] & 0xF) << 8) | d[5]
    return (
        uint_to_float(p_int, P_MIN, P_MAX, 16),
        uint_to_float(v_int, V_MIN, V_MAX, 12),
        uint_to_float(t_int, T_MIN, T_MAX, 12),
        uint_to_float(d[6],  0.0,  60.0,   8),
        uint_to_float(d[7], -40.0, 125.0,  8),
    )

def can2serial(can_id:str, data:str, printing:bool = False)->bytes:
    '''
        can_id: hexadecimal string. 8-digit.
        data: hexadecimal string. 16-digit.
        output: hexadecimal string. 34-digit. 4+8+2+16+4
    '''

    parsed_can_id = parse_hex(can_id)
    parsed_data = parse_hex(data)

    if len(parsed_can_id) != 8:
            raise ValueError(f"can_id must be 8-digit.")
    if len(parsed_data) != 16:
        raise ValueError(f"data must be 16-digit.")
    
    can_id_int = int(parsed_can_id, 16)
    data_int = int(parsed_data, 16)

    frame_header = 0x4154
    frame_tail = 0x0d0a
    extended_frame = 0x08
    dummy_bit = 0b100

    can_id_restructured = can_id_int<<3 | dummy_bit

    output = f"{frame_header:04x}{can_id_restructured:08x}{extended_frame:02x}{data_int:016x}{frame_tail:04x}"

    if printing:
        pretty_output = " ".join(output[i:i+2] for i in range(0, len(output), 2))
        print(f"Output Serial: {pretty_output.lower()}")

    return bytes.fromhex(output)

def parse_hex(hex_input: str) -> str:
    if not isinstance(hex_input, str):
        raise TypeError("Input must be string.")
    
    cleaned = hex_input.replace(" ", "")
    if cleaned.lower().startswith("0x"):
        cleaned = cleaned[2:]
        
    if not cleaned:
        raise ValueError("Invalid Input.")
        
    return cleaned