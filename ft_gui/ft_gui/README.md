Inspired by https://github.com/tasada038/pyqt_ros2_app/tree/master

## To update the UI:

1) Edit with Qt designer

2) Refresh python file:
```
cd <ws>/src/ft_tools_ros2/ft_gui/ft_gui
python3 -m PyQt5.uic.pyuic -x ui_ft_calibration.ui -o ft_calibration_window.py
```
