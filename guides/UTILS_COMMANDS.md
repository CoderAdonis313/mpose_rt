# Util scripts 

## Syntax for using scripts
```python
python -m src.utils.test_contours.py
python -m src.utils.interactive_grab_color_poly.py
```

## Multithreading tutorial
We are using multiple threads instead of multiple process to handle live camera & pose detection separately. Since sharing common objects like images b/w multiple process is difficult, we are using threads for now. 