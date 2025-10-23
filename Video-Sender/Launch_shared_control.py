import subprocess
import time

# List your Python scripts
tele_path = "PiPER_teleoperation.py"
share_path = "PiPER_shared_control.py"

scripts = [tele_path, share_path]

processes = []

for script in scripts:
    # Start each script in a new process
    p = subprocess.Popen(["python3", script])
    processes.append(p)
    time.sleep(1)
# Wait for all to finish (optional)
for p in processes:
    p.wait()
