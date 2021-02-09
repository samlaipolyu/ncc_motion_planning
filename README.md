### Non-Constant Curvature Motion Planning

<hr>
Author: Jiewen Lai, The Hong Kong Polytechnic University<br>
e-mail: jw.lai[at]connect[dot]polyu[dot]hk
<hr>

#### Cite as
- Jiewen Lai (2021). A MATLAB Simulator: Constrained Motion Planning of A Cable-Driven Soft Robot with Non-Constant Curvature Modeling. https://github.com/samlaipolyu/ncc_motion_planning.<br>
- Jiewen Lai, Qingxiang Zhao, and Henry K. Chu. "paper title," Journal Name. vol. xx. no. x. pp. xxxx-xxxx. 2021.

<hr>

#### This file contains the matlab codes of
 - An executable `opt.m` file as the main function.
 - A function file `fwk.m` for the forward kinematics from cable input `x` to end-effector result `p_02`.
 - three function files `fwkeul_alpha.m`,`fwkeul_beta.m`,`fwkeul_gamma.m` for the euler angles derived from the `fwk.m`.
 - A function file `manipulator.m` to detect the collision between preset obstacle and manipulator's body.

<hr>

#### The codes have been tested on MATLAB R2017a and the above versions.

<hr>

#### How-To.

- Download all the files to the local in a single subfolder.
- Customize the setting in `opt.m`, such as 
  1. the desired trajecotry,
  2. setting angle (`line 56`) for constraint 1, or the obstacle parameters (`line 50 - line 54`) for the constraint 2, and
  3. the constraints for the optimization, such as `ub` and `lb` (`line 100 - line 103`)
- Run `opt.m`. It will produce the animation of motion, and the related plots in a few seconds. The computational time depends on various factors, inclduing but not limited to 
  1. the number of nodes of the desired trajectory
  2. the optimization setting
  3. the real-world feasiblity and difficulty of the computed pose of the soft robot
  4. and of course, your computer.
- The main function also provide the `gs=GlobalSearch` method which has been tested. It will compute the same result. No local minima problem.
- The main function also provide a `svd(J{i})` method to compute the optimal lambda for each optimzation step.
- The data shall be smoothed before the implementation to the real motors, because the intially computed cable actuations are sometimes jumpy which are only suitable for simulation. You can play around with the data with the `smoothdata()` function (`'sgolay'` is recommanded).
- The `.mlx` files are the live scripts. They compute the decoupled solution in a symbolic way.
- The image processing code is provided as `depth_cam_obsv.m`. It computes the 3D cooridinate of the POIs using a RealSense D435 depth camera.


  
 
