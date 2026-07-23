import time
from pymavlink import mavutil

def scan_ports():
    ports = ['/dev/ttyTHS1', '/dev/ttyTHS2', '/dev/ttyS0']
    bauds = [57600, 115200, 921600]
    
    print("🔍 MEMULAI BRUTEFORCE SCANNER MAVLINK...")
    for port in ports:
        for baud in bauds:
            print(f"[*] Mencoba {port} dengan baudrate {baud}...")
            try:
                master = mavutil.mavlink_connection(port, baud=baud)
                msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2.0)
                if msg:
                    print(f"✅ BINGO! FC KETEMU DI: {port} DENGAN BAUDRATE {baud}!")
                    print(f"ℹ️ Status: {msg}")
                    return
                else:
                    print("❌ Timeout.")
            except Exception as e:
                print(f"⚠️ Error buka port: {e}")
                
    print("\n💀 KESIMPULAN: MAVLink tidak ditemukan di port manapun!")
    print("1. Pastikan kabel GND Jetson nyambung ke GND FC.")
    print("2. Coba pindah colokan fisik kabel TX/RX ke port TELEM/UART lain di DekeFPV.")

if __name__ == "__main__":
    scan_ports()
