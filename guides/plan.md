# Plan for Realtime Megapose

<b>In words: </b><br>
1. Get a bounding box from your cheap detector
2. if first frame: Run full coarse + refiner pipeline
3. capture this pose into previous pose
4. if following frame: Use this pose to run only refiner 
5. compare bbox from detector & mpose
5. if drift, run full pipeline again


```python
# previous_pose = None

# while video.isOpened():
#     success, frame = video.read()
#     if not success:
#         break

#     observation = create_megapose_observation(
#         frame=frame,
#         camera_intrinsics=K
#     )

#     if previous_pose is None:
#         bbox = detect_object(frame)

#         pose = run_coarse_and_refiner(
#             observation,
#             bbox=bbox
#         )
#     else:
#         pose = run_refiner(
#             observation,
#             initial_pose=previous_pose,
#             n_iterations=1
#         )

#     previous_pose = pose

#     draw_pose_axes(frame, pose, K)
#     output_video.write(frame)
```