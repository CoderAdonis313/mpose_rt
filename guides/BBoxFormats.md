# Bounding Box Formats Comparison

In computer vision, there is no single universal format for bounding boxes. Instead, there are three major standard formats. In all standard formats, the origin point `(0, 0)` is the **top-left corner** of the image.

## Summary Comparison Table

| Format | Structure | Coordinate Type | Common Use |
| :--- | :--- | :--- | :--- |
| **Pascal VOC** | `[x_min, y_min, x_max, y_max]` | Absolute (Pixels) | PyTorch, OpenCV, XML annotations |
| **COCO** | `[x_min, y_min, width, height]` | Absolute (Pixels) | Evaluation metrics, JSON annotations |
| **YOLO** | `[x_center, y_center, width, height]`| Normalized (0 to 1) | YOLO training text files |
