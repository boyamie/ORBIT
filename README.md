# ORBIT: Autonomous Tracking Drone Starter Project

ORBIT is a Python starter project for autonomous tracking with a **DJI Tello** drone.
It combines:

- **Live drone video** via `djitellopy`
- **YOLOv8** person detection via `ultralytics`
- **PID control loops** via `simple-pid`

The goal is to keep a detected person centered in the video frame while adjusting distance smoothly.

## Project Structure

- `/home/runner/work/ORBIT/ORBIT/requirements.txt` – Python dependencies
- `/home/runner/work/ORBIT/ORBIT/config.py` – PID gains, limits, and model settings
- `/home/runner/work/ORBIT/ORBIT/main.py` – Drone connection, video inference, and control loop

## Setup

1. Create and activate a virtual environment:

   ```bash
   python -m venv .venv
   source .venv/bin/activate
   ```

2. Install dependencies:

   ```bash
   pip install -r requirements.txt
   ```

## Run

```bash
python main.py
```

Press **`q`** to stop tracking.

## Notes

- Ensure your computer is connected to the Tello Wi-Fi network before running the script.
- The script includes error handling for connection and streaming failures.
- PID gains in `config.py` are starter values and should be tuned for your environment.
