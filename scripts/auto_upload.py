import sys
import serial.tools.list_ports

try:
    Import("env")
except Exception:
    pass

def find_esp32_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        hwid = getattr(p, "hwid", "") or ""
        # Ignore Bluetooth ports
        if "BTHENUM" in hwid:
            continue
        # Check ESP32-S3 native USB (VID 0x303A, PID 0x1001)
        if "303A:1001" in hwid.upper():
            return p.device
        # Check other common ESP USB-UART bridges (CP210x, CH340, FTDI)
        if any(vid in hwid.upper() for vid in ["10C4:EA60", "1A86:7523", "0403:6001"]):
            return p.device
        # Generic USB serial fallback (has USB in hwid but not Bluetooth)
        if "USB" in hwid.upper():
            return p.device
    return None

try:
    com_port = find_esp32_port()
    if "env" in globals() or "env" in locals():
        config = env.GetProjectConfig()
        section = "env:" + env["PIOENV"]
        
        ota_target = "esp32-waterlevel.local"
        if config.has_option(section, "custom_ota_ip"):
            val = config.get(section, "custom_ota_ip").strip()
            if val:
                ota_target = val

        if com_port:
            print("\n" + "=" * 60)
            print(f"[AUTO-UPLOAD] USB Serial Port detected: {com_port}")
            print("--> Using USB Cable (Serial) for upload")
            print("=" * 60 + "\n")
            env.Replace(UPLOAD_PROTOCOL="esptool")
            env.Replace(UPLOAD_PORT=com_port)
        else:
            print("\n" + "=" * 60)
            print("[AUTO-UPLOAD] No USB Serial Port detected!")
            print(f"--> Switching to Wi-Fi OTA (Target: {ota_target})")
            
            # Detect local IP that routes to ota_target to avoid multi-NIC binding issues on Windows
            local_host_ip = None
            try:
                import socket
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.connect((ota_target, 80))
                local_host_ip = s.getsockname()[0]
                s.close()
            except Exception:
                pass
            
            if local_host_ip:
                print(f"--> Host IP bound to: {local_host_ip}")
                env.Append(UPLOADERFLAGS=["-I", local_host_ip])
                env.Append(UPLOAD_FLAGS=["-I", local_host_ip])
            print("=" * 60 + "\n")
            env.Replace(UPLOAD_PROTOCOL="espota")
            env.Replace(UPLOAD_PORT=ota_target)
    else:
        print(f"[STANDALONE TEST] Found COM Port: {com_port}")
except Exception as e:
    print(f"[AUTO-UPLOAD] Error in auto_upload.py: {e}", file=sys.stderr)
