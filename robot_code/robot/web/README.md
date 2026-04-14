# Robot Web

General Raspberry Pi dashboard for:

- starting and stopping the motion runtime from a browser
- exposing the current `gait-leg-bridge` CLI flags as a form
- starting and stopping the existing camera web page
- embedding the camera page in the same dashboard
- providing a web override controller that can temporarily take priority over UART input

## Files

- `app.py`: Flask dashboard host
- `process_control.py`: subprocess launch and log capture helpers
- `requirements.txt`: dashboard dependency
- `systemd/robot-web.service`: boot-time service for the dashboard

## Run Manually

From the Raspberry Pi:

```bash
cd /home/team6/Desktop/robot/web
python3 -m pip install -r requirements.txt
python3 app.py --host 0.0.0.0 --port 8091
```

Then open:

```text
http://172.26.160.172:8091/
```

## What The Dashboard Controls

Motion runtime:

```bash
python3 -m runtime.main --port /dev/ttyAMA0 --baudrate 115200 --verbose gait-leg-bridge --duration 0.0 --cmd-scale 1000.0 --timeout 0.25 --deadzone 100 --override-file /home/team6/Desktop/robot/web/runtime_override.json
```

Camera page:

```bash
python3 live.py camera --host 0.0.0.0 --port 8080
```

The dashboard runs as its own web service. The runtime and camera stay off until you start them from the page.

## Enable Auto-Hosting At Boot

Copy the service file and enable it:

```bash
sudo cp /home/team6/Desktop/robot/web/systemd/robot-web.service /etc/systemd/system/robot-web.service
sudo systemctl daemon-reload
sudo systemctl enable --now robot-web.service
sudo systemctl status robot-web.service
```

## Notes

- The dashboard hosts on port `8091` by default.
- The camera page hosts on port `8080` by default.
- The dashboard only manages one motion runtime process and one camera process at a time.
- Motion runtime logs and camera logs are tailed directly into the page.
- The web override controller writes a short-lived file override. While active, it wins over UART input. Once released or expired, UART takes back control.
