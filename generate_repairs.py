import os
import subprocess

def main():
    print("STARTED!")
    corrupted_dir = 'corrupted_images'
    repair_dir = 'repair_images'

    if not os.path.exists(repair_dir):
        os.makedirs(repair_dir)

    for filename in os.listdir(corrupted_dir):
        if not filename.lower().endswith('.png'):
            continue

        input_path = os.path.join(corrupted_dir, filename)
        output_filename = os.path.splitext(filename)[0] + '.ppm'
        repair_filename = os.path.splitext(filename)[0] + '.repaired.ppm'
        output_path = os.path.join(repair_dir, output_filename)
        repair_path = os.path.join(repair_dir, repair_filename)
        print("Repairing", input_path)
        try:
            # Call png_mechanic and direct the output to the repair directory
            subprocess.run(['./png_mechanic', input_path, "-o", output_path, "-r", repair_path], check=True)
            print(f'Fixed image saved as {output_path}')
        except subprocess.CalledProcessError as e:
            print(f'Failed to repair {input_path}: {e}')

if __name__ == '__main__':
    main()

