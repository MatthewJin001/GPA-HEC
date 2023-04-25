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
[out] = Algo19_GPAM(bRie,btie,qij,pattern,patternX,patternY,K)
```
where
* ``bRie`` (3x3xn): the rotation matrix of robot pose from effector to base,
* ``btie`` (3xn): the translation vector of robot pose  from effector to base (unit: m),
* ``qij`` (2xnxm): the 2D pixel,
* ``pattern`` (3xm): the point position on the pattern, only used by GPMP,
* ``patternX`` (1x1): the pattern row, used to determine the center point for initialization,
* ``patternY`` (1x1): the pattern cloumn, used to determine the center point for initialization,
* ``K`` (3x3): the camera intrinsic,
* ``out``: the output structure, specifically includes the following
* ``eRc`` (3x3): the rotation part of hand-eye pose from camera to effector,
* ``etc`` (3x1): the translation part of hand-eye pose from camera to effecotor (unit: m),
* ``p`` (3mx1): the marker positons in the base frame, 
* ``rnti1`` (1x1):  the data preparation runtime (unit: seconds),
* ``rnti2`` (1×1)： the total runtime (unit: seconds).

### Demos
Demo codes are included that generate sythetic datasets and run calibrations on the generated datasets.
* To generate a synthetic dataset, run ``main_dataGeneration_SE3.m``.
* To run calibration on the generated dataset, run ``main_solve_AXYB_SE3.m``.

SO(3) case is handled in ``main_dataGeneration_SO3.m`` and ``main_solve_AXYB_SO3.m``

```
       Method          Time        TimeD       Proj      Rec  
    _____________    ________    _________    ______    ______

    {'Tsai'     }      1.7045       1.6613    3.0243     2.723
    {'Park'     }      1.5945       1.5656    3.0215    2.7227
    {'Horaud'   }      1.6124       1.5988    3.0214    2.7223
    {'Liang'    }      1.5536       1.5438    3.0214    2.7223
    {'Li'       }      1.5731       1.5679    2.3647    1.9909
    {'Shah'     }      1.5188       1.5149    1.4898    1.4405
    {'TabbZ1'   }      2.2759       1.5356    1.6917    1.5853
    {'TabbZ2'   }      10.009       1.5189    1.6701    1.5838
    {'TabbR'    }      2.1257       1.4869    1.4113    1.3595
    {'AliX1'    }      3.0857       1.5695    3.3509    3.0197
    {'AliX2'    }      2.6798       1.5229    3.4001    3.1389
    {'AliR1'    }      3.9818       1.5295    1.4353    1.3806
    {'AliR2'    }      2.6223       1.5303    1.4139    1.3612
    {'Zhao'     }      3.3255        1.412    2.7252    2.4904
    {'Wu'       }      1.5477       1.5415    3.4871    3.1516
    {'Sarabandi'}       1.548       1.5413    3.0603    2.7448
    {'GPAS'     }    0.018806    0.0003097    1.4438     1.353
    {'GPAP'     }     0.10938     0.000254    1.4113    1.3595
    {'GPAM'     }    0.048826    0.0004161    1.4275    1.3479
```

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





