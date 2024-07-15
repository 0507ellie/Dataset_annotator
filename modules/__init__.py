import sys
from pathlib import Path
if getattr(sys, 'frozen', False):
    script_dir = Path(sys._MEIPASS)
else:
    script_dir = Path(__file__).resolve().parent
sys.path.append(str(script_dir))

