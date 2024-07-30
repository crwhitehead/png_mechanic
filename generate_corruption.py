"""
noted hard
Repairing corrupted_images/base2_corruption_16556_1877.png
"""

import os
import random

def corrupt_image(image_data, corruption_size, start_position):
    # Create corrupted data by replacing a portion with random bytes
    corrupted_data = bytearray(image_data)
    for i in range(start_position, min(start_position + corruption_size, len(image_data))):
        corrupted_data[i] = random.randint(0, 255)
    return corrupted_data

def main():
    input_dir = 'test_images'
    output_dir = 'corrupted_images'
    num_corruptions = 100

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    for filename in os.listdir(input_dir):
        if not filename.lower().endswith(('.png')):
            continue

        input_path = os.path.join(input_dir, filename)
        with open(input_path, 'rb') as f:
            image_data = f.read()

        image_size = len(image_data)
        for i in range(num_corruptions):
            corruption_size = random.randint(1, 2048)
            start_position = random.randint(0, image_size - corruption_size)

            corrupted_data = corrupt_image(image_data, corruption_size, start_position)
            output_filename = f'{os.path.splitext(filename)[0]}_corruption_{start_position}_{corruption_size}{os.path.splitext(filename)[1]}'
            output_path = os.path.join(output_dir, output_filename)

            with open(output_path, 'wb') as f:
                f.write(corrupted_data)

            print(f'Corrupted image saved as {output_path}')

if __name__ == '__main__':
    main()

