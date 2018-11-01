# InverseKinematics
A new inverse kinematics for biped robot, whose leg has six degrees of freedom orthogonally.

## Prerequisites

The inverse kinematics module depends on Eigen3. Install Eigen3 in advance.

## Parameters

For different-planned robots, the geometry parameters differ. So you have to changes several variables in InverseKinematics.cpp line 14 to 19.

- **upper_leg_length**:  Length of robot's thigh.
- **lower_leg_length**: Length of robot's calf.
- **ankle_from_ground**: When robot is standing vertical to the ground with all leg joints stretched, the distance from intersection of ankle_roll joint and ankle_pitch joint to the ground.
- **half_hip_width**: Half the length of two hip_yaw joints, i.e hip_y_from_origin of left leg.
- **hi_x_from_origin**: The x_offset between hip axis' intersection and body center point.
- **hi_z_from_origin**: The z_offset between hip axis' intersection and body center point. 

## Important function

The constructor of InvKin class should be given your object is right leg or not.

- LegInvKin
  - input: The position(x,y,z) and orientation(r,p,y) of one foot.
  - output: Angles of each joint in the order from up to down.

- CosineTheorem

  - using the law of cosines to get one angle's value in a triangle.

- GetDelta

  - get the angle between two vector(2D or 3D).
