## Simultaneous hand-eye and target parameter estimation by solving 2d-3d generative point alignment problem

## Overview
Solve the 2D-3D generative point alignment problem to estimate both the hand-eye parameter and the parameters of different calibration targets (single marker, mutiple markers and pattern).
![mainFig](https://user-images.githubusercontent.com/32157027/58380789-21da9e00-7fbe-11e9-9cf7-e81a5b515652.png)

**_Figure_**: Formulations relating geometrical transformation for calibration (a) Hand-Eye Calibration (b) Robot-World-Hand-Eye Calibration


## How to use
### Dependencies
The code is dependent on Matlab and some of its toolkits such as Computer Vision Toolbox.
### Main Instructions
To run the calibration, call
```
[X, Y] = solveAXYB_SE3(A,B,alpha,param)
```
where
* ``A, B`` are the measurement datasets, each of which is in size of ``4 X 4 X n`` (for n measurements).
* ``alpha`` is weight factor of translation error; e.g., transration error of 1.0mm is equally weighted to 1 radian rotational error given ``alpha = 1.0`` (when length unit in data is mm).
* ``param`` is a struct of algorithm parameters. Declare by ``param = defaultParam()`` and set ``param.globalOptMethod = 2`` for stochastic global optimization. See  ``instruction.docx`` for more details.
* ``X, Y`` are the calibration results.
### Demos
Demo codes are included that generate sythetic datasets and run calibrations on the generated datasets.
* To generate a synthetic dataset, run ``main_dataGeneration_SE3.m``.
* To run calibration on the generated dataset, run ``main_solve_AXYB_SE3.m``.

SO(3) case is handled in ``main_dataGeneration_SO3.m`` and ``main_solve_AXYB_SO3.m``


## Open Dataset
A synced sequence of camera image and poses from end-effector can be downloaded from https://github.com/zarathustr/hand_eye_data.

## Reference
Jin, G., Yu, X., Chen, Y., Li, J. (2023), Simultaneous hand-eye and target parameter estimation
%            by solving 2d-3d generative point alignment problem, submitted to IEEE Trans. Instrum. Meas.

## Video
To do.

## Usefull links for hand-eye calibration
* Kenji Koide and Emanuele Menegatti, General Hand-Eye Calibration based on Reprojection Error Minimization, IEEE Robotics and Automation Letters/ICRA2019, https://github.com/koide3/st_handeye_graph
* Wu, J., Sun, Y., Wang, M., Liu*, M. (2019) Hand-eye Calibration: 4D Procrustes Analysis Approach. IEEE Trans. Instrum. Meas, https://github.com/MatthewJin001/hand_eye_SO4




## Contact

Gumin Jin, Department of Automation, Shanghai Jiao Tong University, Shanghai, jingumin@sjtu.edu.cn





