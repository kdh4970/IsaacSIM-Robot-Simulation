print("Checking python modules...")
try:
    from PyQt5.QtCore import Qt
except ImportError or ModuleNotFoundError:
    print("Insatlling PyQt5...",end="")
    import sys,subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "pyqt5"])
try:
    import pynput
except ImportError or ModuleNotFoundError:
    print("Insatlling pynput...",end="")
    import sys,subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "pynput"])
print("Done")