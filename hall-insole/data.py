import pandas as pd
import os
import re

# --- Podešavanje ---
OUTPUT_FILENAME = 'data_out.csv'
# -------------------

def is_ser_file(fname):
    return re.fullmatch(r'\d+_ser\.csv', fname)

def get_ros_filename(ser_fname):
    return ser_fname.replace('_ser.csv', '_ros.csv')

def main():
    # Putanja do foldera 'data' jedan nivo iznad skripte
    root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    data_dir = os.path.join(root_dir, 'data')
    output_path = os.path.join(data_dir, OUTPUT_FILENAME)

    result_rows = []

    # Pronađi sve _ser.csv fajlove i sortiraj ih po broju
    ser_files = [f for f in os.listdir(data_dir) if is_ser_file(f)]
    ser_files.sort(key=lambda name: int(re.match(r'(\d+)_ser\.csv', name).group(1)))

    for ser_fname in ser_files:
        ros_fname = get_ros_filename(ser_fname)
        ser_path = os.path.join(data_dir, ser_fname)
        ros_path = os.path.join(data_dir, ros_fname)

        if not os.path.exists(ros_path):
            print(f"Fajl {ros_fname} ne postoji, preskačem par {ser_fname}.")
            continue

        try:
            ser_df = pd.read_csv(ser_path)
            ros_df = pd.read_csv(ros_path)

            if not all(col in ser_df.columns for col in ['x', 'y', 'z']):
                print(f"{ser_fname} nema kolone x, y, z – preskačem.")
                continue

            required_ros_cols = [
                'force_x', 'force_y', 'force_z',
                'torque_x', 'torque_y', 'torque_z'
            ]
            if not all(col in ros_df.columns for col in required_ros_cols):
                print(f"{ros_fname} nema sve potrebne kolone – preskačem.")
                continue

            row = {
                'filename': ser_fname,
                'mag_x_mean': ser_df['x'].mean(),
                'mag_x_std': ser_df['x'].std(),
                'mag_y_mean': ser_df['y'].mean(),
                'mag_y_std': ser_df['y'].std(),
                'mag_z_mean': ser_df['z'].mean(),
                'mag_z_std': ser_df['z'].std(),
            }

            for col in required_ros_cols:
                row[f'{col}_mean'] = ros_df[col].mean()
                row[f'{col}_std'] = ros_df[col].std()

            result_rows.append(row)

        except Exception as e:
            print(f"Greška pri obradi para {ser_fname} i {ros_fname}: {e}")

    if result_rows:
        result_df = pd.DataFrame(result_rows)
        result_df.to_csv(output_path, index=False)
        print(f"Završeno. Rezultati zapisani u {output_path}")
    else:
        print("Nema validnih parova fajlova za obradu.")

if __name__ == '__main__':
    main()
