import math

def clamp(value, min_value, max_value):
    """
    Membatasi (clamp) nilai agar tidak kurang dari min_value 
    dan tidak lebih dari max_value. (Anti-Drift / Speed Limit)
    """
    return max(min_value, min(max_value, value))

def calculate_distance(x1, y1, x2, y2):
    """
    Menghitung jarak euclidean (INS distance) antara 2 titik.
    Dipakai untuk blind punch-through nembus lorong gawang 
    atau hitung jarak over-shoot (kebablasan).
    """
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def get_stutter_creep_speed(timeout_counter, base_speed=0.3, active_ticks=10, cycle_ticks=20):
    """
    Fungsi khusus untuk "Stutter Creep" (rem-gas-rem-gas).
    Secara default: 1 detik maju (10 ticks), 1 detik ngerem (10 ticks).
    Ini maksa pitch drone rata biar kamera bawah bisa nyapu lantai.
    """
    if timeout_counter % cycle_ticks < active_ticks:
        return base_speed
    return 0.0
