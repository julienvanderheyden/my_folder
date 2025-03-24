# RSONs
Collection of robots in "Robot Structured Object Notation" format.

This format is still in development, does not have a publicly available parser, and is subject to change.

The motivation behind this format is twofold:
- To develop a portable definition format in plaintext for robots which goes beyond kinematic/inertial properties, allowing the definition of springs, dampers, inerters and other components.
- To develop a format which is not based upon XML (such as URDF), because I don't like it.

## Franka panda

Frank panda inertial parameters are taken from https://github.com/frankaemika/franka_ros/blob/develop/franka_description/robots/common/inertial.yaml