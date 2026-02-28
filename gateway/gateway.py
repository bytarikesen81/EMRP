import struct
import time
import serial
import RPi.GPIO as GPIO
from typing import List, Tuple, Optional

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


def main():
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

	# CODE BEFORE LOOP HERE #
    #Use actual IMU ranges chosen in dataset training and testing
    accel_range_g = 2
    gyro_range_dps = 250

    try:
        while True:
            win = recv_imu_window(
                ser,
                A=96,
                samples_per_frame=4,   # set 1 if you truly send 1 sample per frame
                expect_addh=0x00,
                expect_addl=0x00,
                expect_chan=0x06,
                timeout_s=3.0,
            )

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

    finally:
        uart.close()
        
main()

if _name_ == "_main_":
    main()