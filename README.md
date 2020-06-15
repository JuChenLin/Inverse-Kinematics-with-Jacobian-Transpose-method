# Inverse-Kinematics-with-Jacobian-Transpose-method
### **Implement the Inverse Kinematics (IK) on a linkage of four cuboids based on Jacobian Transpose method**


### Functions

##### Inverse Kinematics (IK)
Inverse Kinematics is to find the values of the joint pose (position + orientation) that produce a desired end-effector location. In this project, the linkage has 3 joints, which is represented by 1 cube as base and 3 cuboids as bones, and 9 degree of freedoms. End each bone is associated with 3 DOF, i.e. the rotation angles along x, y and z axis. For any 3 DOF joint, rotations is used in the order: y-axis, z-axis, x-axis. The initial pose vector for each bone is (𝜃𝑦, 𝜃𝑧, 𝜃𝑥) = (0.0, 30.0, 0.0), with all numbers in degrees.

##### Jacobian Transpose method
There are several methods for solving IK problems, including pseudoinverse, Jacobian transpose, Jacobian Inverse, Damped Least Squares methods, etc. The Jacobian Transpose has advantages of less computationally expensive, which means faster calculation, and no matrix inversions. However it is of relatively poor quality.

##### GUI
The program support interactively setting the target end effector position on GUI. The initial position is (3.0, 8.0, 3.0). A green cube is drawn at the target position to represent it. Also, the GUI showes real-time pose vectors and end position of the linkage.


### API
OpenGL (Open Graphics Library)


### Results


