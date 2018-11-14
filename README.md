# InverseKinematics
A new inverse kinematics for humanoid biped robot, whose leg has six degrees of freedom orthogonally.

## Prerequisites

The inverse kinematics module depends on Eigen3. Please install Eigen3 in advance.

---



## Parameters

### InvKin

For different-planned robots, the geometry parameters differ. So you have to changes several variables in InverseKinematics.cpp line 14 to 19.

- **upper_leg_length**:  Length of robot's thigh.
- **lower_leg_length**: Length of robot's calf.
- **ankle_from_ground**: When robot is standing vertical to the ground with all leg joints stretched, the distance from intersection of ankle_roll joint and ankle_pitch joint to the ground.
- **half_hip_width**: Half the length of two hip_yaw joints, i.e hip_y_from_origin of left leg.
- **hi_x_from_origin**: The x_offset between hip axis' intersection and body center point.
- **hi_z_from_origin**: The z_offset between hip axis' intersection and body center point. 



---



## Important function

The constructor of InvKin class should be given whether your object is right leg or not.

### InvKin

This class can be used for calculate the inverse kinematics solution of single leg.

- InvKin
  - Constructor of this class, in which you should give the target leg is right or left.

- LegInvKin
  - Input: The position(x,y,z) and orientation(r,p,y) of one foot.
  - Output: Angles of each joint in the order from up to down.
- CosineTheorem

  - Using the law of cosines to get one angle's value in a triangle.
  - This function have been moved into dmotion_math.cpp
- GetDelta
  - Get the angle between two vector(2D or 3D).
  - This function have been moved into dmotion_math.hpp

### WholeBodyIK 

This class can be used for calculate the inverse kinematics solution of two legs when the two feet are set in a plane. The plane orientation can be set when you initialize a object.

- WholeBodyIK
  - Constructor of this class, in which you should give the feet distance in x, y, z directions, left leg relative to right leg.
- GetIKResult
  - Input: The position of body center relative to the middle point of between two feet.
  - Output: The servo angles of 12 servos, from left to right, form up to down.
- ChangeFootPos
  - Change the relative foot position.

---



## Test

Clone this repository to your computer, and build the project.

```
git clone https://github.com/hannbusann/InverseKinematics.git
cd InverseKinematics
mkdir build 
cd build
cmake ..
make
./InvKin_test
```

