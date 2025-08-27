python lerobot/scripts/control_robot.py \
  --robot.type=so101 \
  --control.type=record \
  --control.fps=16 \
  --control.single_task="Grasp an object." \
  --control.repo_id=ktkd/eval_so101_office_test_20250817 \
  --control.tags='["tutorial"]' \
  --control.warmup_time_s=5 \
  --control.episode_time_s=30 \
  --control.reset_time_s=30 \
  --control.num_episodes=10 \
  --control.push_to_hub=false \
  --control.policy.path=outputs/whhe/plugin_test_20250815/checkpoints/040000/pretrained_model
  # --control.policy.path=outputs/whhe/plugin_diffusion_20250815/checkpoints/080000/pretrained_model
  # --control.policy.path=outputs/whhe/plugin_test_20250815/checkpoints/040000/pretrained_model
