


#### Terminal 1 (Local PC)
```sh
cd /home/saurabh/Development/quest3_streamer
source .venv/bin/activate
python3 -m src.launch.webxr_bridge \
  --mode ingress \
  --host 0.0.0.0 \
  --port 9999 \
  --cert certs/cert.pem \
  --key certs/key.pem \
  --forward-url wss://192.168.30.11:9998 \
  --forward-insecure
```

#### Terminal 2 (Local PC)
```sh
cd /home/saurabh/Development/quest3_streamer
python3 web/https_server.py 8000 --cert certs/cert.pem --key certs/key.pem
```

#### Terminal 1 (Remote PC)

```sh
cd /home/saurabh/Development/quest3_streamer
source .venv/bin/activate
source /opt/ros/humble/setup.bash
python3 -m src.launch.webxr_bridge \
  --mode remote-receiver \
  --host 0.0.0.0 \
  --port 9998

```

#### Terminal 2 (Remote PC)

```sh
cd /home/saurabh/Development/quest3_streamer
bash scripts/run_openarm_teleop.sh
```

#### Terminal 2 (Remote PC) for recording episode

```sh
cd /home/saurabh/Development/quest3_streamer
source .venv-lerobot-v21/bin/activate
./scripts/run_acone_teleop.sh \
  --record \
  --dataset-root datasets \
  --dataset-repo-id local/quest3-acone \
  --task "Teleoperate acone to complete the task" \
  --recording-fps 30 \
  --max-episodes 10
```

#### Terminal 3 (Remote PC) for visualizing the dataset
```sh
cd /home/saurabh/Development/quest3_streamer
source .venv-lerobot-v21/bin/activate
./scripts/visualize_lerobot_dataset.sh \
  --repo-id local/quest3-acone  \
  --root datasets/local/quest3-acone \
  --episode-index 0
```