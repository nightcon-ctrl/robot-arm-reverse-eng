# Mate guide (SolidWorks / Onshape / Fusion)

STEP carries positions, not mates, so the assembly opens in its pose but is not yet movable.
Every part's origin is on its own joint axis, so each joint below is two or three mates.
Fix `01_optical_plate`, then add:

## Fixed to the base (no motion between these)
- `02_spacer` bottom face coincident with `01_optical_plate` top face; spacer and plate hole pattern concentric with the yaw axis (Z).
- `03_yaw_disc`, `04_base_enclosure`, `05_yaw_actuator`, `06_upright`, `07_shoulder_mount`, `08_shoulder_flange`
  are one rigid group that rotates about Z. Mate them to each other (coincident faces), then add the **yaw joint**:
  - **Concentric**: `03_yaw_disc` outer cylinder with `02_spacer` outer cylinder (yaw axis).
  - **Coincident**: `03_yaw_disc` bottom face with `02_spacer` top face (the model leaves a 2.5 mm gap there, so use a distance mate of 2.5 mm, or 0 if you add a bearing).
  - Leave the rotation free (or add an angle mate for yaw, range ±180°).
- `04_base_enclosure` bottom face coincident with `03_yaw_disc` top face.
- `08_shoulder_flange` bore concentric with the `07_shoulder_mount` shoulder axis; its back face (+Y) coincident with the mount's -Y face. Its four screw heads stand proud of the front face (-Y).

## Shoulder joint (axis along Y at height 98.70 mm)
Group that rotates: `09_shoulder_actuator`, `10_connector_beam`, `11_beam_standoff`, `12_elbow_actuator` (all rigid to each other).
- **Concentric**: `09_shoulder_actuator` bore with `08_shoulder_flange` bore.
- **Coincident**: `09_shoulder_actuator` back face (+Y) with `08_shoulder_flange` front face (-Y).
- **Coincident**: `10_connector_beam` back face with `09_shoulder_actuator` front boss face; beam bore concentric with the actuator bore.
- Leave the rotation free, or add an angle mate between the beam's long axis and the plate's X axis (convention: 0° = +X, positive toward +Z).

## Elbow joint (axis along Y, 102.00 mm along the beam)
Group that rotates: `13_elbow_flange`, `14_forearm`, `15_elbow_mount`.
- `11_beam_standoff` back face coincident with `12_elbow_actuator` front boss face; both concentric with the beam's elbow-end bore.
- `12_elbow_actuator` bore **concentric** with `13_elbow_flange` bore; `12_elbow_actuator` back face **coincident** with the flange's front face (the four screw heads stand proud of it at R25.5, outside the housing radius, so they do not collide).
- `14_forearm` front face coincident with `13_elbow_flange` back face; elbow-end bore concentric.
- `15_elbow_mount` front face coincident with `14_forearm` back face; mount axis through the elbow bore.
- Leave the rotation free (angle convention as above, forearm angle measured in the same sense).

## Y stack (camera side is -Y), from the annotations
beam -48.70 .. -43.58 | shoulder actuator .. -25.49 | shoulder flange .. -21.08 | shoulder mount .. 19.83
elbow: standoff -43.58 .. -37.79 | actuator .. -19.70 | flange .. -15.29 | forearm .. -10.17 | mount .. 29.83
