#Runs on boot (via the wro-start-button systemd service) and waits for the physical start
#button before launching the actual run script. Button: signal wire on GPIO16 (physical pin
#36), ground wire on physical pin 34.
import os
import re
import subprocess
import sys

from gpiozero import Button

SRC_DIR = os.path.dirname(os.path.abspath(__file__))
THIS_FILE = os.path.abspath(__file__)

RUN_SCRIPT_NAME = "open_challenge.py"   # <-- change this filename to pick what the button launches

def get_run_script():
    #re-reads THIS line from disk on every press (instead of using the value captured at
    #startup) so editing RUN_SCRIPT_NAME above takes effect on the next button press without
    #needing to restart the systemd service
    with open(THIS_FILE) as f:
        for line in f:
            match = re.match(r'RUN_SCRIPT_NAME\s*=\s*"([^"]+)"', line)
            if match:
                return os.path.join(SRC_DIR, match.group(1))
    return os.path.join(SRC_DIR, RUN_SCRIPT_NAME)

start_button = Button(16, bounce_time=0.05)

while True:
    print("Waiting for start button press...")
    start_button.wait_for_press()
    run_script = get_run_script()
    print("Button pressed - launching", run_script)

    #blocks here until the run script exits (window closed with 'q', or it finishes the run
    #on its own), then loops back to wait for the next press
    subprocess.run([sys.executable, run_script], cwd=SRC_DIR)

    print("Run finished.")