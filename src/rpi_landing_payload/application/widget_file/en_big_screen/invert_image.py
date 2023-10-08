#!/usr/bin/python3

from PIL import Image
import sys
import os

def invert_png(input_path):
    try:
        # Open the input image
        img = Image.open(input_path)

        # Invert the image
        inverted_img = Image.new('RGBA', img.size)
        for x in range(img.width):
            for y in range(img.height):
                r, g, b, a = img.getpixel((x, y))
                inverted_img.putpixel((x, y), (255 - r, 255 - g, 255 - b, a))

        # Get the input image's filename without the extension
        filename, ext = os.path.splitext(input_path)

        # Construct the output filename by appending "_inverted" to the input filename
        output_path = filename + "_inverted.png"

        # Save the inverted image
        inverted_img.save(output_path)

        print("Image inversion completed and saved as", output_path)

    except Exception as e:
        print("An error occurred:", str(e))

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python invert_image.py <input_image_path>")
        sys.exit(1)

    input_path = sys.argv[1]

    invert_png(input_path)

