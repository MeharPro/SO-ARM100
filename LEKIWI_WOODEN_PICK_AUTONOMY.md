# LeKiwi Wooden-Piece Autonomy

This is the shortest realistic path from the current working LeKiwi teleop setup to a camera-driven policy that picks up a wooden piece and closes the gripper.

## What This Is

This is imitation learning, not a one-click scripted grasp.

The policy will learn from demonstrations where you:
- use the leader arm to approach the wooden piece
- close the gripper around it
- lift it cleanly

The cameras are the key input. If the policy is going to pick based on vision, the wooden piece must be clearly visible in the same camera views during both recording and evaluation.

## Current Assumptions

- Pi host IP: `192.168.40.135`
- LeKiwi robot id: `follow-mobile`
- Leader id: `leader`
- Leader is connected to the Mac
- The Pi runs `lekiwi_host`
- The LeKiwi front and wrist cameras are working

## 1. Start The Pi Host

On the Pi:

```bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
cd ~/lerobot
python -m lerobot.robots.lekiwi.lekiwi_host --host.connection_time_s=3600 --robot.id=follow-mobile
```

## 2. Record Demonstrations

On the Mac:

```zsh
source /Users/meharkhanna/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
cd /Users/meharkhanna/robot-arm
python scripts/lekiwi_record_wood_pick_dataset.py \
  --remote-host 192.168.40.135 \
  --robot-id follow-mobile \
  --dataset-repo-id meharkhanna/lekiwi_wood_pick \
  --dataset-root ~/lerobot-datasets/lekiwi_wood_pick \
  --num-episodes 50 \
  --episode-time-s 20 \
  --reset-time-s 10
```

Data collection guidance:
- Record at least `50` good episodes to start.
- Change the wooden piece position across episodes.
- Keep the cameras fixed.
- Use consistent grasp behavior.
- Perform the task while watching the camera views, not by looking directly at the robot.
- Make sure the episode includes the gripper closing around the wooden piece.

### Free-Teach Alternative

If you want to hand-guide the follower arm directly instead of using the leader arm:

On the Pi, run the passive-arm host:

```bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
cd ~/lerobot
python scripts/lekiwi_free_teach_host.py \
  --robot.id=follow-mobile \
  --host.connection_time_s=3600
```

This disables arm torque so you can move the arm by hand.

Then on the Mac, record with:

```zsh
source /Users/meharkhanna/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
cd /Users/meharkhanna/robot-arm
python scripts/lekiwi_record_wood_pick_dataset.py \
  --remote-host 192.168.40.135 \
  --robot-id follow-mobile \
  --dataset-repo-id meharkhanna/lekiwi_wood_pick \
  --dataset-root ~/lerobot-datasets/lekiwi_wood_pick \
  --num-episodes 50 \
  --episode-time-s 20 \
  --reset-time-s 10 \
  --arm-free-teach
```

In this mode:
- arm actions are taken from the follower's current observed joint positions
- base actions still come from keyboard teleop
- this is the best way to include wheel behavior while hand-guiding the arm

## 3. Train The Policy

On the Mac:

```zsh
source /Users/meharkhanna/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
cd /Users/meharkhanna/lerobot

lerobot-train \
  --dataset.repo_id=meharkhanna/lekiwi_wood_pick \
  --dataset.root=/Users/meharkhanna/lerobot-datasets/lekiwi_wood_pick \
  --policy.type=act \
  --policy.device=mps \
  --output_dir=/Users/meharkhanna/lerobot/outputs/train/act_lekiwi_wood_pick \
  --job_name=act_lekiwi_wood_pick \
  --policy.push_to_hub=false
```

Notes:
- `mps` is for Apple silicon.
- If you use a Linux/NVIDIA machine later, switch to `--policy.device=cuda`.
- The best checkpoint will usually be:
  - `/Users/meharkhanna/lerobot/outputs/train/act_lekiwi_wood_pick/checkpoints/last/pretrained_model`

## 4. Run The Policy On The Robot

Keep the Pi host running.

On the Mac:

```zsh
source /Users/meharkhanna/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
cd /Users/meharkhanna/robot-arm
python scripts/lekiwi_eval_wood_pick_policy.py \
  --remote-host 192.168.40.135 \
  --robot-id follow-mobile \
  --policy-path /Users/meharkhanna/lerobot/outputs/train/act_lekiwi_wood_pick/checkpoints/last/pretrained_model \
  --dataset-repo-id meharkhanna/eval_lekiwi_wood_pick \
  --dataset-root ~/lerobot-datasets/eval_lekiwi_wood_pick \
  --num-episodes 10 \
  --episode-time-s 20
```

This runs the trained policy and also saves evaluation episodes locally so you can inspect what the robot actually did.

## 5. What To Expect

The gripper close is not separately hard-coded in this pipeline. It is part of the learned action sequence.

## Wheels

Yes, the wheels can be part of training.

The cleanest way is:
- train arm motion from leader teleop or free-teach arm poses
- train base motion from keyboard commands at the same time

That works because LeKiwi's action space already includes:
- arm joint positions
- `x.vel`
- `y.vel`
- `theta.vel`

I would not rely on physically pushing the base by hand as the main supervision signal. It is much noisier than keyboard base commands.

If the policy fails to close at the right time, the usual fixes are:
- collect more demonstrations
- reduce variation at first
- keep lighting and camera framing stable
- make sure the object is large and visible in both front and wrist views
- start with one wooden piece in one workspace region, then expand

## 6. Faster Alternative

If the task is always the same wooden piece in nearly the same place, a classical scripted vision grasp can be faster than training:
- detect the wooden piece in the front or wrist camera
- move to a fixed pre-grasp offset
- descend
- close gripper
- lift

That path is simpler if you want a narrow single-scene demo.

The learning path above is better if you want the robot to generalize across positions and camera observations.
