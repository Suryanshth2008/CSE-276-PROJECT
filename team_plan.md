# Two-person implementation plan

## Member 1 — Vision/perception
- Prepare representative outdoor video clips and note terrain/lighting conditions.
- Run the baseline and inspect masks.
- Compare simple thresholding against a suitable pretrained terrain-segmentation model.
- Measure path-region quality on a small labelled set.
- Document failure cases (shadows, low light, vegetation, gravel, objects blending into ground).

## Member 2 — Navigation/evaluation
- Review the steering decision and define safe stop conditions.
- Add an explicit navigation state machine: INIT, PATH_FOUND, PATH_UNCERTAIN, OBSTACLE, CAMERA_ERROR, COMPLETE.
- Create repeatable test-run definitions.
- Calculate success rate, mean path deviation, failures per run, and processing speed where ground truth supports them.
- Build the dashboard only after the pipeline and logs work.

## Shared interface
Per-frame perception output should eventually include:
- traversable-region mask
- path/waypoint estimate
- obstacle detections and confidence
- uncertainty/confidence
- frame timestamp

Navigation consumes that output and returns:
- decision/state
- reason code
- target waypoint
- timestamp

Do not treat an image-space left/right suggestion as a calibrated motor command.
