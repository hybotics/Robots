#!/usr/bin/python
import time
import sys

# Import CircuitPlayground class from the circuitplayground.py in the same directory.
from circuitplayground import CircuitPlayground


# Grab the serial port from the command line parameters.
if len(sys.argv) != 2:
    print('ERROR! Must specify the serial port as command line parameter.')
    sys.exit(-1)
port = sys.argv[1]

# Connect to Circuit Playground board on specified port.
board = CircuitPlayground(port)

# Define a rainbow of colors:
colors = [ (255,   0,   0),  # Red color (components are red, green, blue)
           (255, 128,   0),  # Orange
           (255, 255,   0),  # Yellow
           (  0, 255,   0),  # Green
           (  0,   0, 255),  # Blue
           ( 75,   0, 130),  # Indigo
           (143,   0, 255) ] # Violet

# Animate moving the colors across the pixels 100 times / 10 seconds.
print('Animating pixels for 10 seconds...')
for offset in range(100):
    # Go through each pixel and set its color based on its position and the
    # current offset.  Constrain these values to fall within the list of colors.
    for i in range(10):
        # Find the color for this pixel.
        color = colors[(i+offset)%len(colors)]
        # Set the pixel color.
        board.set_pixel(i, color[0], color[1], color[2])
    # Push the updated colors out to the pixels (this will make the pixels change
    # their color, the previous set_pixel calls just change the memory and not
    # the pixels).
    board.show_pixels()
    # Sleep for a bit between iterations.
    time.sleep(0.1)

# Clear all the pixels to turn them off.
board.clear_pixels()
board.show_pixels()  # Make sure to call show after clear!
time.sleep(0.1)      # Small delay to make sure board connection doesn't close
                     # before previous command is processed.

# Close Firmata board connection when done.
board.close()
