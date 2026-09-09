# Templates


## Read videos using cv2
```python
import cv2

# Open the default camera
cam = cv2.VideoCapture(0)

# Get the default frame width and height
frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('output.mp4', fourcc, 20.0, (frame_width, frame_height))

while True:
    ret, frame = cam.read()

    # Write the frame to the output file
    out.write(frame)

    # Display the captured frame
    cv2.imshow('Camera', frame)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) == ord('q'):
        break

# Release the capture and writer objects
cam.release()
out.release()
cv2.destroyAllWindows()
```


## Function naming conventions generally
* **Trailing Underscore (`function_`):** A single trailing underscore is conventionally used to prevent naming conflicts with Python's reserved keywords and built-in functions.


* **Single Leading Underscore (`_function`):** A single leading underscore acts as a polite warning that a method or variable is intended for internal use only, serving as a weak "private" indicator.


* **Double Underscores (Dunder):** Double leading underscores trigger name mangling to prevent accidental subclass overriding, while double leading and trailing underscores (like `__init__`) are strictly reserved for Python's special core behaviors.