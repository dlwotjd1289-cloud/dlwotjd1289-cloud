from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
for name in ('pac_common','pac_perception','pac_planning','pac_robot'):
    sys.path.insert(0,str(ROOT/'ros2_ws'/'src'/name))
