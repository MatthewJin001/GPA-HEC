## Simultaneous hand-eye and target parameter estimation by solving 2D-3D generative point alignment problem

## Overview
Hand-eye calibration is further studied as a 2D-3D generative point alignment (GPA) problem. For multi-point, single-point and patterned calibration targets, GPAM, GPAS and GPAP are proposed to simultaneously estimate the hand-eye and the target parameters, and solved by a initialization-to-refinement structure. A general initialization method based on a single-point sequence is novelly proposed, which definetely gets rid of the dependence on pose. The perturbation-basd refinement is optimizaed using analytical Jacobians and point sparsity. 

![mainFig](https://github.com/MatthewJin001/GPA-HEC/blob/main/figure/gitPic.png)

**_Figure_**: Visual representation of the hand-eye calibration of GPAs (a) GPAM (b) GPAS (c) GPAP


## How to use
### Dependencies
It works well on MATLAB R2023a. The GPAs themselves do not use any solvers, while comparison algorithms and evaluation need the solvers in Optimization Toolbox.

### Main Instructions
To run the GMAM calibration, call
```
[X, Y] = solveAXYB_SE3(A,B,alpha,param)
[out] = Algo19_GPAM(bRie,btie,qij,pattern,patternX,patternY,K)
```
where
* ``A, B`` are the measurement datasets, each of which is in size of ``4 X 4 X n`` (for n measurements).
* ``alpha`` is weight factor of translation error; e.g., transration error of 1.0mm is equally weighted to 1 radian rotational error given ``alpha = 1.0`` (when length unit in data is mm).
* ``param`` is a struct of algorithm parameters. Declare by ``param = defaultParam()`` and set ``param.globalOptMethod = 2`` for stochastic global optimization. See  ``instruction.docx`` for more details.
* ``X, Y`` are the calibration results.

* ``bRie`` (3x3xn): the rotation matrix of robot pose from effector to base,
* ``btie`` (3xn): the translation vector of robot pose  from effector to base (unit: m),
* ``qij`` (2xnxm): the 2D pixel,
* ``pattern`` (3xm): the point position on the pattern, only used by GPMP,
* ``patternX`` (1x1): the pattern row, used to determine the center point for initialization,
* ``patternY`` (1x1): the pattern cloumn, used to determine the center point for initialization,
* ``K`` (3x3): the camera intrinsic,

### Demos
Demo codes are included that generate sythetic datasets and run calibrations on the generated datasets.
* To generate a synthetic dataset, run ``main_dataGeneration_SE3.m``.
* To run calibration on the generated dataset, run ``main_solve_AXYB_SE3.m``.

SO(3) case is handled in ``main_dataGeneration_SO3.m`` and ``main_solve_AXYB_SO3.m``


## Open Dataset
A synced sequence of camera image and poses from end-effector can be downloaded from https://github.com/zarathustr/hand_eye_data.

![mainFig](https://github.com/MatthewJin001/GPA-HEC/blob/main/figure/2.png)

**_Figure_**: Visual representation of the GPA hand-eye calibration (a) GPAM (b) GPAS (c) GPAP

## Reference
Jin, G., Yu, X., Chen, Y., Li, J. (2023), Simultaneous hand-eye and target parameter estimation
%            by solving 2d-3d generative point alignment problem, submitted to IEEE Trans. Instrum. Meas.

## Video
https://github.com/MatthewJin001/GPA-HEC

## Usefull links for hand-eye calibration
* Kenji Koide and Emanuele Menegatti, General Hand-Eye Calibration based on Reprojection Error Minimization, IEEE Robotics and Automation Letters/ICRA2019, https://github.com/koide3/st_handeye_graph
* Wu, J., Sun, Y., Wang, M., Liu*, M. (2019) Hand-eye Calibration: 4D Procrustes Analysis Approach. IEEE Trans. Instrum. Meas, https://github.com/MatthewJin001/hand_eye_SO4




## Contact

Gumin Jin, Department of Automation, Shanghai Jiao Tong University, Shanghai, jingumin@sjtu.edu.cn





