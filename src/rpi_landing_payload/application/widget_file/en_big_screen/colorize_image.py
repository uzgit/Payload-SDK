#!/usr/bin/python3

from PIL import Image
import sys
import os

def invert_and_multiply_red(input_path):
    try:
        # Open the input image
        image = Image.open(input_path)

        # Invert the image
        colorized_image = Image.new('RGBA', image.size)
        for x in range(image.width):
            for y in range(image.height):
                r, g, b, a = image.getpixel((x, y))

                if( r < 150 and g < 150 and b < 150 ):
                    r = 255

                colorized_image.putpixel((x, y), (r, g, b, a))

        # Get the input image's filename without the extension
        filename, ext = os.path.splitext(input_path)

        # Construct the output filename by appending "_inverted" to the input filename
        output_path = filename + "_colorized.png"

        # Save the inverted image
        colorized_image.save(output_path)


    except Exception as e:
        print("An error occurred:", str(e))

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python invert_multiply_red.py <input_image_path>")
        sys.exit(1)

    input_path = sys.argv[1]

    invert_and_multiply_red(input_path)

