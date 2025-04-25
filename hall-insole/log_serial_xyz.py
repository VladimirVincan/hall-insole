import serial
import csv
import time
import re
import os
import sys

# --- Podesi ovde samo baudrate i trajanje snimanja ---
BAUDRATE = 115200
MEASUREMENT_DURATION_SECONDS = 10
# ------------------------------------------------------

def main():
    # Provera da li je COM port prosleđen kao argument
    if len(sys.argv) < 2:
        print("Greska: Morate proslediti COM port kao argument, npr:")
        print("python log_serial_xyz.py COM3")
        sys.exit(1)

    port = sys.argv[1]
    csv_user_filename = input("Unesi ime za CSV fajl (bez ekstenzije, npr. 'merenja_xyz'): ").strip()

    # Odredi putanju za 'data' folder jedan nivo iznad skripte
    root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    data_dir = os.path.join(root_dir, 'data')
    os.makedirs(data_dir, exist_ok=True)

    csv_filename = os.path.join(data_dir, f'{csv_user_filename}.csv')

    try:
        ser = serial.Serial(port, BAUDRATE, timeout=1)
        print(f"Povezano na {port} @ {BAUDRATE} baud.")
        print(f"Snimanje podataka u fajl: {csv_filename}")
        print(f"Skripta ce se automatski zaustaviti nakon {MEASUREMENT_DURATION_SECONDS} sekundi...")

        start_time = time.time()

        with open(csv_filename, mode='w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['timestamp', 'x', 'y', 'z'])

            while True:
                # Provera da li je prošlo 10 sekundi
                if time.time() - start_time > MEASUREMENT_DURATION_SECONDS:
                    print(f"Isteklo {MEASUREMENT_DURATION_SECONDS} sekundi. Zavrsavam snimanje...")
                    break

                line = ser.readline().decode('utf-8').strip()

                if 'x :' in line:
                    x = float(re.search(r"[-+]?\d*\.\d+|\d+", line).group())
                elif 'y :' in line:
                    y = float(re.search(r"[-+]?\d*\.\d+|\d+", line).group())
                elif 'z :' in line:
                    z = float(re.search(r"[-+]?\d*\.\d+|\d+", line).group())
                    timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
                    writer.writerow([timestamp, x, y, z])
                    csvfile.flush()
                    print(f"{timestamp} -> x: {x}, y: {y}, z: {z}")

    except serial.SerialException as e:
        print(f"Greska sa serijskom komunikacijom: {e}")
    except KeyboardInterrupt:
        print("\nPrekid programa od strane korisnika.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serijski port zatvoren.")

if __name__ == '__main__':
    main()
