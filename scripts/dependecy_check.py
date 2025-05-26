print("Checking python modules...")
DEPENDENCY_LIST = ["pynput", "numpy"]
import sys,subprocess

## Check PyQt5
try:
    from PyQt5.QtCore import Qt
except ImportError or ModuleNotFoundError:
    print("Insatlling PyQt5...",end="")
    import sys,subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "pyqt5"])

## Check other dependencies
for dependency in DEPENDENCY_LIST:
    try:
        exec(f"import {dependency}")
    except ImportError or ModuleNotFoundError:
        print(f"Insatlling {dependency}...",end="")
        subprocess.check_call([sys.executable, "-m", "pip", "install", dependency])

print("Done")