# Plan for Realtime Megapose


```python
previous_pose = None

while video.isOpened():
    success, frame = video.read()
    if not success:
        break

    observation = create_megapose_observation(
        frame=frame,
        camera_intrinsics=K
    )

    if previous_pose is None:
        bbox = detect_object(frame)

        pose = run_coarse_and_refiner(
            observation,
            bbox=bbox
        )
    else:
        pose = run_refiner(
            observation,
            initial_pose=previous_pose,
            n_iterations=1
        )

    previous_pose = pose

    draw_pose_axes(frame, pose, K)
    output_video.write(frame)
```