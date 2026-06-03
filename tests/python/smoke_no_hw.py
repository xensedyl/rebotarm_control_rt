"""手动冒烟：无硬件时构造 RobotArm 应抛出干净的 Python 异常（非 panic）。"""
import warnings

warnings.simplefilter("ignore")
from rebotarm_control_rt.actuator import RobotArm

try:
    arm = RobotArm()
    print("UNEXPECTED: constructed without hardware:", arm)
except Exception as e:  # noqa: BLE001
    print("OK graceful error:", type(e).__name__, "-", str(e)[:100])
