## Simultaneous Hand-eye And Target Estimation By 2D-3D Generative Point Alignment

## Overview
Jin, G., Yu, X., Chen, Y., Zhang, L., Li, J. (2023), Simultaneous hand-eye and target estimation by 2D-3D generative point alignment, submitted to IEEE Trans. Autom. Sci. Eng.

Hand-eye calibration in this paper is further studied as a 2D-3D generative point alignment (GPA) problem. For multi-point, single-point and patterned calibration targets, GPAM, GPAS and GPAP are proposed to simultaneously estimate the hand-eye and the target parameters. These formulations are solved by a initialization-to-refinement structure. A general initialization method based on a single-point sequence is proposed for the first time, which definetely gets rid of the dependence on pose. The pose-perturbation refinement is optimizaed using analytical Jacobian matrix and inherent sparsity. 

![mainFig](https://github.com/MatthewJin001/GPA-HEC/blob/main/figure/gitPic.png)

**_Figure_**: Visual representation of the hand-eye calibration of GPA methods (a) GPAM (b) GPAS (c) GPAP.


## How to use
### Dependencies
It works well on MATLAB R2023a. The GPA methods themselves are solver-free, while compared algorithms and evaluation need the solvers in Optimization Toolbox.

### Main Instructions
To run the GPAM calibration, call
```
[out] = Algo19_GPAM(bRie,btie,qij,pattern,patternX,patternY,K)
```
where
* ``bRie`` (3x3xn): the rotation matrix of robot pose from effector to base,
* ``btie`` (3xn): the translation vector of robot pose  from effector to base (unit: m),
* ``qij`` (2xnxm): the 2D pixel,
* ``pattern`` (3xm): the point position on the pattern, only used by GPAP,
* ``patternX`` (1x1): the pattern row, used to determine the center point for initialization,
* ``patternY`` (1x1): the pattern cloumn, used to determine the center point for initialization,
* ``K`` (3x3): the camera intrinsics,
* ``out``: the output structure, specifically includes the following
* ``eRc`` (3x3): the rotation matrix of hand-eye pose from camera to effector,
* ``etc`` (3x1): the translation vector of hand-eye pose from camera to effecotor (unit: m),
* ``p`` (3mx1): the feature point positons in the base frame, 
* ``rnti1`` (1x1):  the data preparation runtime (unit: seconds),
* ``rnti2`` (1×1)： the total runtime (unit: seconds).

### Demos
Demo ``main`` contains the calibration and evaluation of multiple methods. run ``main.m``, the results will be stored in ``result.xlsx``. The calibration and the evaluation results of the normal dataset in the paper are as follows
```
      Method          tx         ty         tz        Rx         Ry        Rz  
    _____________    _______    _______    ______    _______    ______    ______

    {'Tsai'     }    -7.7698     -39.71    60.601    -30.714    26.023    38.065
    {'Park'     }    -7.7004    -39.678    60.612    -30.741    26.032    38.059
    {'Horaud'   }    -7.6994    -39.677    60.612    -30.741    26.031    38.061
    {'Liang'    }    -7.6995    -39.677    60.612    -30.741     26.03    38.061
    {'Li'       }     -9.581    -39.506    56.023    -30.792    25.879    38.002
    {'Shah'     }    -6.4477    -43.536    53.069    -30.794    25.878    38.002
    {'TabbZ1'   }    -7.3139      -42.3    54.278    -30.843    25.946    37.932
    {'TabbZ2'   }    -6.5372    -43.287    54.756    -30.708    25.831     38.12
    {'TabbR'    }    -6.7196    -43.616    51.889    -30.742    25.875    38.036
    {'AliX1'    }    -7.0119    -38.716    60.887    -30.909    25.881    38.196
    {'AliX2'    }    -6.2126    -39.046    61.142    -30.961    25.883    38.157
    {'AliR1'    }    -7.4246     -44.16    52.158    -30.644    25.848     38.07
    {'AliR2'    }    -6.8326    -43.298    51.485     -30.75    25.878    37.983
    {'Zhao'     }    -6.3412    -39.347    57.654    -30.857     25.83    38.081
    {'Wu'       }    -8.9322    -39.555    62.226    -30.741    26.026    38.065
    {'Sarabandi'}    -7.6559    -39.671    60.618     -30.74    25.929    38.148
    {'GPAS'     }    -6.8563    -42.375    52.069    -30.772     25.92     37.97
    {'GPAP'     }    -6.7196    -43.616     51.89    -30.742    25.875    38.036
    {'GPAM'     }    -7.2773    -43.033    51.989    -30.749    25.854    38.015
```
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
The self-made datasets, JAKA (Normal), Wrinkled and Small, along with the public datasets, DENSE and KUKA, will be organized and uploaded to the ``dataset`` folder in the near feature.

![mainFig](https://github.com/MatthewJin001/GPA-HEC/blob/main/figure/2.png)

**_Figure_**: Image of the dataset (a) JAKA (Normal) (b) Wrinkled (c) Small.

## Video
Video record for the hand-eye calibration dataset is on https://www.youtube.com/watch?v=udUMbf67ntw
![mainFig](https://github.com/MatthewJin001/GPA-HEC/blob/main/figure/exp.jpg)
**_Figure_**: Experimental configuration.
## Reference
* Sarabandi S, Porta J M, Thomas F. Hand-eye calibration made easy through a closed-form two-stage method[J]. IEEE Robotics and Automation Letters, 2022, 7(2): 3679-3686, https://github.com/MatthewJin001/Hand-Eye-Calibration
* Wu J, Sun Y, Wang M, et al. Hand-eye calibration: 4-D procrustes analysis approach[J]. IEEE Transactions on Instrumentation and Measurement, 2019, 69(6): 2966-2981, https://github.com/MatthewJin001/hand_eye_SO4
* Tabb A, Ahmad Yousef K M. Solving the robot-world hand-eye (s) calibration problem with iterative methods[J]. Machine Vision and Applications, 2017, 28(5-6): 569-590, https://github.com/MatthewJin001/RWHEC-Tabb-AhmadYousef
* Ali I, Suominen O, Gotchev A, et al. Methods for simultaneous robot-world-hand–eye calibration: A comparative study[J]. Sensors, 2019, 19(12): 2837, https://github.com/MatthewJin001/RWHE-Calib


## Contact

Gumin Jin, Department of Automation, Shanghai Jiao Tong University, Shanghai, jingumin@sjtu.edu.cn





