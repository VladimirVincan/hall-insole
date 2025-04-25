import serial
import csv
import time
import re
import os
import sys

# --- Podesi ovde samo baudrate ---
BAUDRATE = 115200
# ----------------------------------

def main():
    # Provera da li je COM port prosleđen kao argument
    if len(sys.argv) < 2:
        print("Greska: Morate proslediti COM port kao argument, npr:")
        print("python log_serial_xyz.py COM3")
        sys.exit(1)

    port = sys.argv[1]  # Uzimamo COM port iz argumenta
    csv_user_filename = input("Unesi ime za CSV fajl (bez ekstenzije, npr. 'merenja_xyz'): ").strip()

    # Odredi putanju za 'data' folder jedan nivo iznad skripte
    root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    data_dir = os.path.join(root_dir, 'data')
    os.makedirs(data_dir, exist_ok=True)  # Ako ne postoji, kreiraj 'data'

    # Formiraj punu putanju do CSV fajla
    csv_filename = os.path.join(data_dir, f'{csv_user_filename}.csv')

    try:
        ser = serial.Serial(port, BAUDRATE, timeout=1)
        print(f"Povezano na {port} @ {BAUDRATE} baud.")
        print(f"Snimanje podataka u fajl: {csv_filename}")

        with open(csv_filename, mode='w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['timestamp', 'x', 'y', 'z'])

            print("Pocetak snimanja podataka... (prekini sa Ctrl+C)")

            while True:
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
