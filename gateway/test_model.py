#!/usr/bin/env python3
import time
import json
import csv
import argparse
import serial
import RPi.GPIO as GPIO
from typing import List, Tuple, Optional
import numpy as np
from smbus2 import SMBus
import tensorflow as tf
import firebase_admin
from firebase_admin import credentials, db

#LoRa Globals
PIN_LORA_M0 = 11
PIN_LORA_M1 = 13
PIN_LORA_AUX = 15

lora_params = bytes([0xC0, 0x00, 0x00, 0x1A, 0x06, 0x40])
cmd_version = bytes([0xC3, 0xC3, 0xC3])
cmd_params = bytes([0xC1, 0xC1, 0xC1])

uart = serial.Serial(
		port= "/dev/serial0",
		baudrate = 9600,
		timeout = 1
	)
MAGIC = b"TE"

# IMU Window Frame format:
# ADDH(1) ADDL(1) CHAN(1) MAGIC(2) FID(1) FC(1) PAYLOAD(48)
HEADER_LEN = 1 + 1 + 1 + 2 + 1 + 1
PAYLOAD_LEN = 48
FRAME_LEN = HEADER_LEN + PAYLOAD_LEN

#HELPERS
def raise_error(msg:str, code: int, final_act, *args, **kwargs):
	print(f"[ERROR:], {msg}", file=sys.stderr)
	final_act(*args, **kwargs)
	raise SystemExit(code)
	
def wait_until_pin(pin, state, timeout_s: float) -> bool:
	t0 = time.monotonic()
	while (time.monotonic() - t0) < timeout_s:
		if GPIO.input(pin) == state: 
			return True
		time.sleep(0.002)
	return False

# ---------------- MPU6050 minimal I2C driver ----------------

MPU_ADDR_DEFAULT = 0x68

REG_PWR_MGMT_1     = 0x6B
REG_CONFIG         = 0x1A
REG_SMPLRT_DIV     = 0x19
REG_GYRO_CONFIG    = 0x1B
REG_ACCEL_CONFIG   = 0x1C
REG_ACCEL_XOUT_H   = 0x3B  # 14-byte burst


class MPU6050:
    def __init__(self, bus_id=1, addr=MPU_ADDR_DEFAULT):
        self.addr = addr
        self.bus = SMBus(bus_id)

    def write_reg(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val & 0xFF)

    def read_block(self, reg, n):
        return self.bus.read_i2c_block_data(self.addr, reg, n)

    @staticmethod
    def _to_i16(msb, lsb):
        v = (msb << 8) | lsb
        return v - 65536 if v & 0x8000 else v

    def initialize(self, accel_range="2g", gyro_range="250dps"):
        # Wake up device
        self.write_reg(REG_PWR_MGMT_1, 0x00)
        time.sleep(0.05)

        # Moderate DLPF (helps noise)
        self.write_reg(REG_CONFIG, 0x03)

        # Sample rate divider (software loop enforces Fs)
        self.write_reg(REG_SMPLRT_DIV, 0x00)

        accel_map = {"2g": 0x00, "4g": 0x08, "8g": 0x10, "16g": 0x18}
        gyro_map  = {"250dps": 0x00, "500dps": 0x08, "1000dps": 0x10, "2000dps": 0x18}

        if accel_range not in accel_map:
            raise ValueError("Unsupported accel_range")
        if gyro_range not in gyro_map:
            raise ValueError("Unsupported gyro_range")

        self.write_reg(REG_ACCEL_CONFIG, accel_map[accel_range])
        self.write_reg(REG_GYRO_CONFIG, gyro_map[gyro_range])

    def read_raw6(self):
        b = self.read_block(REG_ACCEL_XOUT_H, 14)
        ax = self._to_i16(b[0], b[1])
        ay = self._to_i16(b[2], b[3])
        az = self._to_i16(b[4], b[5])
        gx = self._to_i16(b[8], b[9])
        gy = self._to_i16(b[10], b[11])
        gz = self._to_i16(b[12], b[13])
        return ax, ay, az, gx, gy, gz

    def close(self):
        try:
            self.bus.close()
        except Exception:
            pass


# ---------------- TFLite engine ----------------
class TFLiteEngine:
    def __init__(self, model_path: str, show_io: bool = False):
        self.interpreter = tf.lite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()
        self.in_details = self.interpreter.get_input_details()[0]
        self.out_details = self.interpreter.get_output_details()[0]
        self.in_idx = self.in_details["index"]
        self.out_idx = self.out_details["index"]

        if show_io:
            print("TFLite input details:", self.in_details)
            print("TFLite output details:", self.out_details)

    def predict(self, x: np.ndarray) -> np.ndarray:
        # x must be float32 with shape (1,96,6)
        self.interpreter.set_tensor(self.in_idx, x.astype(np.float32))
        self.interpreter.invoke()
        y = self.interpreter.get_tensor(self.out_idx)
        return np.asarray(y[0], dtype=np.float32).reshape(-1)


# ---------------- Helpers ----------------
def cleanup():
	uart.close()
	GPIO.cleanup()
     
def int16_to_float_acc(raw: int, accel_range_g: int = 2) -> float:
    # MPU6050: ±2g => 16384 LSB/g, ±4g => 8192, ±8g => 4096, ±16g => 2048
    lsb_per_g = {2: 16384.0, 4: 8192.0, 8: 4096.0, 16: 2048.0}[accel_range_g]
    return raw / lsb_per_g


def int16_to_float_gyro(raw: int, gyro_range_dps: int = 250) -> float:
    # MPU6050: ±250 => 131 LSB/(°/s), ±500 => 65.5, ±1000 => 32.8, ±2000 => 16.4
    lsb_per_dps = {250: 131.0, 500: 65.5, 1000: 32.8, 2000: 16.4}[gyro_range_dps]
    return raw / lsb_per_dps


def _read_exact(ser: serial.Serial, n: int, timeout_s: float) -> bytes:
    """Read exactly n bytes or raise TimeoutError."""
    deadline = time.time() + timeout_s
    buf = bytearray()
    while len(buf) < n:
        if time.time() > deadline:
            raise TimeoutError(f"Timeout while reading {n} bytes (got {len(buf)})")
        chunk = ser.read(n - len(buf))
        if chunk:
            buf.extend(chunk)
    return bytes(buf)


def _sync_to_magic(ser: serial.Serial, timeout_s: float) -> None:
    """
    Resync by searching for MAGIC in stream and positioning read so that
    next reads start at frame boundary (best-effort).
    """
    deadline = time.time() + timeout_s
    window = bytearray()

    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        window += b
        if len(window) > 64:
            window = window[-64:]

        idx = window.find(MAGIC)
        if idx == -1:
            continue

        # MAGIC is at offset 3 in a well-formed frame:
        # [0]=ADDH [1]=ADDL [2]=CHAN [3..4]=MAGIC
        # If it is found that MAGIC somewhere in the sliding window, attempt to align.
        # window are the tail of previous stream; after detecting, just return.
        return

    raise TimeoutError("Failed to sync to MAGIC within timeout")


def recv_imu_window(
    ser: serial.Serial,
    A: int = 96,
    samples_per_frame: int = 4,   # 48 payload / (6*2) = 4 samples
    expect_addh: int = 0x00,
    expect_addl: int = 0x00,
    expect_chan: int = 0x06,
    timeout_s: float = 2.0,
) -> List[Tuple[int, int, int, int, int, int]]:
    """
    Receives a full IMU window and returns it as a list of A samples.
    Each sample is (ax, ay, az, gx, gy, gz) in int16 raw.

    Frame checks:
    - ADDH/ADDL/CHAN match
    - MAGIC == b"TE"
    - FC sequence (0..frame_count-1)
    - Consistent FID across frames
    """
    if (PAYLOAD_LEN % (6 * 2)) != 0:
        raise ValueError("PAYLOAD_LEN is not a multiple of one IMU sample (12 bytes).")

    frame_count = (A + samples_per_frame - 1) // samples_per_frame
    out: List[Tuple[int, int, int, int, int, int]] = [None] * A  # type: ignore

    # Try to sync first (best-effort)
    _sync_to_magic(ser, timeout_s=timeout_s)

    fid_expected: Optional[int] = None
    samples_written = 0

    for fc_expected in range(frame_count):
        # Read one full frame
        frame = _read_exact(ser, FRAME_LEN, timeout_s=timeout_s)

        addh, addl, chan = frame[0], frame[1], frame[2]
        magic = frame[3:5]
        fid = frame[5]
        fc = frame[6]
        payload = frame[7:7 + PAYLOAD_LEN]

        # Validate header
        if (addh != expect_addh) or (addl != expect_addl) or (chan != expect_chan) or (magic != MAGIC):
            raise ValueError(
                f"Header mismatch: ADDH/ADDL/CHAN/MAGIC = "
                f"{addh:02X}/{addl:02X}/{chan:02X}/{magic!r}"
            )

        # Validate FID continuity
        if fid_expected is None:
            fid_expected = fid
        elif fid != fid_expected:
            raise ValueError(f"FID mismatch: got {fid}, expected {fid_expected}")

        # Validate FC ordering
        if fc != fc_expected:
            raise ValueError(f"FC mismatch: got {fc}, expected {fc_expected}")

        # Parse payload: little-endian 6x int16 per sample
        # Each sample is 12 bytes -> '<hhhhhh'
        offset = 0
        for _ in range(samples_per_frame):
            if samples_written >= A:
                break
            ax, ay, az, gx, gy, gz = struct.unpack_from("<hhhhhh", payload, offset)
            out[samples_written] = (ax, ay, az, gx, gy, gz)  # type: ignore
            samples_written += 1
            offset += 12

    if samples_written != A:
        raise ValueError(f"Incomplete window: got {samples_written} samples, expected {A}")

    # Return the window safter the transfer is successful
    return out

# Confirmed output order:
LABELS = ["move", "rest", "shake"]

def load_norm_json(path: str):
    """
    Expects JSON with channel_mean and channel_std arrays of length 6.
    """
    with open(path, "r") as f:
        obj = json.load(f)

    # Try a few common key names to be robust
    for mean_key in ["channel_mean", "mean", "means"]:
        if mean_key in obj:
            channel_mean = obj[mean_key]
            break
    else:
        raise KeyError("Could not find channel_mean in normalization.json")

    for std_key in ["channel_std", "std", "stds"]:
        if std_key in obj:
            channel_std = obj[std_key]
            break
    else:
        raise KeyError("Could not find channel_std in normalization.json")

    mean = np.array(channel_mean, dtype=np.float32).reshape(1, 1, 6)
    std  = np.array(channel_std, dtype=np.float32).reshape(1, 1, 6)

    if mean.shape[-1] != 6 or std.shape[-1] != 6:
        raise ValueError("Normalization arrays must have length 6")

    # Avoid division by zero
    std = np.where(std == 0.0, 1e-6, std)
    return mean, std

def window_summary(win_i16: np.ndarray) -> str:
    w = win_i16.astype(np.int32)
    mn = w.min(axis=0)
    mx = w.max(axis=0)
    mean = w.mean(axis=0)
    return (
        f"AX[{mn[0]},{mx[0]}] AY[{mn[1]},{mx[1]}] AZ[{mn[2]},{mx[2]}] "
        f"GX[{mn[3]},{mx[3]}] GY[{mn[4]},{mx[4]}] GZ[{mn[5]},{mx[5]}] "
        f"MEAN={mean.round(1).tolist()}"
    )

def classify(probs: np.ndarray):
    i = int(np.argmax(probs))
    return LABELS[i], float(probs[i])


# ---------------- Main ----------------
def main():
    #Parse initial arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True, help="Path to .tflite model")
    ap.add_argument("--norm_json", required=True, help="Path to normalization.json containing mean/std (6 values each)")
    ap.add_argument("--bus", type=int, default=1, help="I2C bus id (default: 1)")
    ap.add_argument("--addr", type=lambda x: int(x, 0), default=0x68, help="MPU6050 address (default: 0x68)")
    ap.add_argument("--fs", type=float, default=100.0, help="Sampling frequency in Hz (default: 100)")
    ap.add_argument("--A", type=int, default=96, help="Window length (default: 96)")
    ap.add_argument("--K", type=int, default=96, help="Ring buffer length (default: 6)")
    ap.add_argument("--stride", type=int, default=96, help="Stride in samples (default: 96)")
    ap.add_argument("--log", default="", help="Optional CSV log file path (e.g., outputs.csv)")
    ap.add_argument("--show_io", action="store_true", help="Print TFLite I/O tensor details once")
    args = ap.parse_args()

    #Initialize LoRa control ports (M0-M1, AUX)
    GPIO.setmode(GPIO.BOARD)
	GPIO.setwarnings(False)
	GPIO.setup(PIN_LORA_M0, GPIO.OUT)
	GPIO.setup(PIN_LORA_M1, GPIO.OUT)
	GPIO.setup(PIN_LORA_AUX, GPIO.IN, pull_up_down = GPIO.PUD_UP)	
	
	#Wait until AUX is high (Module is ready)
	if not wait_until_pin(PIN_LORA_AUX, GPIO.HIGH, 1.0):
		raise_error("LoRa module is busy!", -1, cleanup())
	else:
		print("LoRa module is available")
	
	#Set module mode (11) sleep
	GPIO.output(PIN_LORA_M0, GPIO.HIGH)
	GPIO.output(PIN_LORA_M1, GPIO.HIGH)
	if not wait_until_pin(PIN_LORA_AUX, GPIO.HIGH, 1.0):
		raise_error("Cannot set parameters", -1, cleanup())
	else:
		print("Parameters are set")
	
	#Send lora configuration parameters
	uart.reset_input_buffer()
	print("TX:", lora_params.hex(" "))
	uart.write(lora_params)
	uart.flush()
	time.sleep(0.1)
	
	#Validate lora configuration parameters
	uart.reset_input_buffer()
	print("TX:", cmd_params.hex(" "))
	uart.write(cmd_params)
	uart.flush()
	rx = uart.read(6)
	if rx != lora_params:
		raise_error("Unable to validate configuration parameters", -1, cleanup())
	else: 
		print("Config parameters:", rx.hex(" "))
		time.sleep(0.1)
	
	#Set module mode (00) transceiver
	if not wait_until_pin(PIN_LORA_AUX, GPIO.HIGH, 1.0):
		raise_error("LoRa module is busy!", -1, cleanup())
	else:
		print("LoRa module is available to transmit")
	GPIO.output(PIN_LORA_M0, GPIO.LOW)
	GPIO.output(PIN_LORA_M1, GPIO.LOW)
	time.sleep(0.01)

    #INIT FIREBASE SERVER
    #Fetch the service account key
    cred = credentials.Certificate("serviceAccountKey.json")

    #Initialize the app with a service account then grant admin privileges
    firebase_admin.initialize_app(cred, {
        'databaseURL': "https://xiao-pet-tracker-controller-default-rtdb.europe-west1.firebasedatabase.app"
    })
    
    #Get imu reference
    controller_ref = db.reference("controllers/1")

    if not args.model.lower().endswith(".tflite"):
        raise ValueError("Model must be a .tflite file")

    if args.A != 96:
        print("Warning: The model is confirmed for A=96. Make sure your model input matches A.")

    # Load global normalization
    channel_mean, channel_std = load_norm_json(args.norm_json)
    # channel_mean/std shape: (1,1,6)

    # Init IMU
    mpu = MPU6050(bus_id=args.bus, addr=args.addr)
    mpu.initialize(accel_range="2g", gyro_range="250dps")

    # Init model
    engine = TFLiteEngine(args.model, show_io=args.show_io)

    # Optional CSV logging
    csv_f = None
    csv_w = None
    if args.log:
        csv_f = open(args.log, "w", newline="")
        csv_w = csv.writer(csv_f)
        csv_w.writerow(["timestamp", "window_id", "move_prob", "rest_prob", "shake_prob", "predicted", "confidence"])

    A = args.A
    fs = args.fs
    K = args.K
    period = 1.0 / fs

    print("Live TFLite inference started.")
    print(f"Model: {args.model}")
    print(f"Normalization: {args.norm_json} (global mean/std)")
    print(f"Fs={fs} Hz, A={A}, stride={args.stride}")
    print("Output order: move, rest, shake")
    if args.log:
        print(f"CSV logging enabled: {args.log}")
    print("Press Ctrl+C to stop.\n")

    try:
        next_t = time.perf_counter()

        while True:
            win = recv_imu_window(
                ser,
                A=96,
                samples_per_frame=4,
                expect_addh=0x00,
                expect_addl=0x00,
                expect_chan=0x06,
                timeout_s=3.0,
            )

            """
            ax, ay, az, gx, gy, gz = win[0]

            ax_f = int16_to_float_acc(ax, accel_range_g)
            ay_f = int16_to_float_acc(ay, accel_range_g)
            az_f = int16_to_float_acc(az, accel_range_g)

            gx_f = int16_to_float_gyro(gx, gyro_range_dps)
            gy_f = int16_to_float_gyro(gy, gyro_range_dps)
            gz_f = int16_to_float_gyro(gz, gyro_range_dps)

            print(
                f"Data received! : "
                f"ax={ax_f:.4f}g, ay={ay_f:.4f}g, az={az_f:.4f}g, "
                f"gx={gx_f:.2f}dps, gy={gy_f:.2f}dps, gz={gz_f:.2f}dps"
            )
            """
            # Enforce sampling rate
            now = time.perf_counter()
            if now < next_t:
                time.sleep(next_t - now)
            next_t += period

            # Global normalization (per-channel)
            win_f32 = win.astype(np.float32).reshape(1, A, 6)
            x = (win_f32 - channel_mean) / channel_std  # (1,96,6) float32

            # Inference (already softmax)
            probs = engine.predict(x)
            if probs.size != 3:
                raise RuntimeError(f"Expected 3 outputs, got {probs.size}")

            move_prob, rest_prob, shake_prob = float(probs[0]), float(probs[1]), float(probs[2])
            pred, conf = classify(probs)

            ts = time.time()

            print(f"[Window {window_id}] {window_summary(win_i16)}")
            print(f"  move_prob={move_prob:.4f}  rest_prob={rest_prob:.4f}  shake_prob={shake_prob:.4f}")
            print(f"  predicted={pred} ({conf*100:.1f}%)")
            print("-" * 90)
            
            #Send the updated status to database
            controller_ref.update({
                "move_prob": move_prob,
                "shake_prob": shake_prob,
                "rest_prob": rest_prob,
                "predicted": pred,
                "timestamp": str(ts)
            })

            if csv_w is not None:
                csv_w.writerow([f"{ts:.3f}", window_id, f"{move_prob:.6f}", f"{rest_prob:.6f}",
                                f"{shake_prob:.6f}", pred, f"{conf:.6f}"])
                csv_f.flush()

            window_id += 1

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        mpu.close()
        if csv_f:
            csv_f.close()


if __name__ == "__main__":
    main()
