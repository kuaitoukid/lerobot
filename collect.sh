python lerobot/scripts/control_robot.py \
  --robot.type=so101 \
  --control.type=record \
  --control.fps=16 \
  --control.single_task="Pick pen to black box." \
  --control.repo_id=ktkd/pick_place_pen \
  --control.tags='["so101","test"]' \
  --control.warmup_time_s=5 \
  --control.episode_time_s=10 \
  --control.reset_time_s=10 \
  --control.num_episodes=5 \
  --control.resume=false \
  --control.push_to_hub=false # true


python lerobot/scripts/control_robot.py `
  --robot.type=so101 `
  --control.type=record `
  --control.fps=16 `
  --control.single_task="Pick red pen to black box." `
  --control.repo_id=ktkd/pick_place_red_pen `
  --control.warmup_time_s=5 `
  --control.episode_time_s=15 `
  --control.reset_time_s=5 `
  --control.num_episodes=2 `
  --control.resume=true `
  --control.push_to_hub=false
