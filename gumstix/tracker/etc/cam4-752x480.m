% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 634.352641465268334 ; 630.822846781609087 ];

%-- Principal point:
cc = [ 368.437429485020402 ; 267.000622900340943 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.402746134272660 ; 0.159232807468770 ; 0.002262734193287 ; 0.000186151611409 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 2.115703622279684 ; 2.286483968934125 ];

%-- Principal point uncertainty:
cc_error = [ 5.240772248847373 ; 4.128093442905377 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.011328210215833 ; 0.029513459134721 ; 0.001243648510605 ; 0.001059706020163 ; 0.000000000000000 ];

%-- Image size:
nx = 752;
ny = 480;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 19;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 3.086218e+00 ; 5.850861e-02 ; -2.846245e-01 ];
Tc_1  = [ -1.063612e+03 ; 9.672325e+02 ; 5.356744e+03 ];
omc_error_1 = [ 8.454701e-03 ; 2.013251e-03 ; 1.430840e-02 ];
Tc_error_1  = [ 4.458719e+01 ; 3.517397e+01 ; 2.489170e+01 ];

%-- Image #2:
omc_2 = [ 3.024692e+00 ; -1.055914e-01 ; 8.581959e-02 ];
Tc_2  = [ -2.435059e+03 ; 1.237557e+03 ; 5.101343e+03 ];
omc_error_2 = [ 8.928313e-03 ; 1.943095e-03 ; 1.521207e-02 ];
Tc_error_2  = [ 4.286355e+01 ; 3.520592e+01 ; 3.476856e+01 ];

%-- Image #3:
omc_3 = [ 2.838064e+00 ; 1.019062e-01 ; -2.903983e-01 ];
Tc_3  = [ -2.688559e+03 ; 6.647397e+02 ; 5.430135e+03 ];
omc_error_3 = [ 7.685530e-03 ; 3.289829e-03 ; 1.351161e-02 ];
Tc_error_3  = [ 4.505497e+01 ; 3.709230e+01 ; 3.348544e+01 ];

%-- Image #4:
omc_4 = [ -2.799562e+00 ; 9.477888e-01 ; 2.534221e-01 ];
Tc_4  = [ 4.268396e+02 ; 1.801274e+03 ; 8.310427e+03 ];
omc_error_4 = [ 1.304263e-02 ; 3.881949e-03 ; 1.876754e-02 ];
Tc_error_4  = [ 6.902634e+01 ; 5.444818e+01 ; 3.780857e+01 ];

%-- Image #5:
omc_5 = [ -1.762695e+00 ; 1.516173e+00 ; 7.897276e-01 ];
Tc_5  = [ 4.079451e+02 ; 1.106331e+03 ; 6.254822e+03 ];
omc_error_5 = [ 4.844362e-03 ; 6.643778e-03 ; 9.897389e-03 ];
Tc_error_5  = [ 5.146182e+01 ; 4.067774e+01 ; 1.824849e+01 ];

%-- Image #6:
omc_6 = [ 2.225665e+00 ; -3.818961e-01 ; -1.604471e+00 ];
Tc_6  = [ 2.193405e+02 ; 1.921785e+03 ; 7.872764e+03 ];
omc_error_6 = [ 8.653097e-03 ; 5.724865e-03 ; 9.918949e-03 ];
Tc_error_6  = [ 6.543021e+01 ; 5.143326e+01 ; 3.149473e+01 ];

%-- Image #7:
omc_7 = [ -2.399596e+00 ; -4.286089e-01 ; 1.595647e+00 ];
Tc_7  = [ -2.147605e+02 ; 1.169599e+01 ; 9.239308e+03 ];
omc_error_7 = [ 9.535850e-03 ; 4.964352e-03 ; 1.019969e-02 ];
Tc_error_7  = [ 7.654081e+01 ; 6.025668e+01 ; 2.934968e+01 ];

%-- Image #8:
omc_8 = [ -2.425257e+00 ; 2.909389e-01 ; -1.131875e+00 ];
Tc_8  = [ -2.163971e+03 ; 8.229297e+02 ; 7.124686e+03 ];
omc_error_8 = [ 8.149179e-03 ; 4.425992e-03 ; 1.143640e-02 ];
Tc_error_8  = [ 5.980244e+01 ; 4.784827e+01 ; 3.987798e+01 ];

%-- Image #9:
omc_9 = [ -2.341906e+00 ; 7.502276e-01 ; -1.095598e+00 ];
Tc_9  = [ -1.838897e+03 ; 1.963311e+03 ; 7.375263e+03 ];
omc_error_9 = [ 8.206519e-03 ; 4.713339e-03 ; 1.147437e-02 ];
Tc_error_9  = [ 6.218333e+01 ; 4.905005e+01 ; 4.011708e+01 ];

%-- Image #10:
omc_10 = [ -2.618731e+00 ; 1.324720e-01 ; -1.542856e+00 ];
Tc_10  = [ -9.594893e+02 ; 9.495180e+02 ; 6.035282e+03 ];
omc_error_10 = [ 8.288836e-03 ; 5.397569e-03 ; 1.207695e-02 ];
Tc_error_10  = [ 5.011235e+01 ; 3.984378e+01 ; 3.386745e+01 ];

%-- Image #11:
omc_11 = [ 1.762711e+00 ; -1.535436e+00 ; -1.070685e+00 ];
Tc_11  = [ 1.796665e+03 ; 1.234670e+03 ; 5.362022e+03 ];
omc_error_11 = [ 7.294960e-03 ; 5.920976e-03 ; 9.629930e-03 ];
Tc_error_11  = [ 4.605741e+01 ; 3.612215e+01 ; 3.018710e+01 ];

%-- Image #12:
omc_12 = [ -2.365486e+00 ; 9.309792e-01 ; 1.177631e+00 ];
Tc_12  = [ -2.778914e+02 ; 2.196978e+03 ; 7.569166e+03 ];
omc_error_12 = [ 6.969541e-03 ; 6.099819e-03 ; 1.123635e-02 ];
Tc_error_12  = [ 6.332841e+01 ; 5.002405e+01 ; 2.773919e+01 ];

%-- Image #13:
omc_13 = [ -1.593154e+00 ; -1.328277e+00 ; -9.413335e-01 ];
Tc_13  = [ -1.296650e+03 ; -4.146853e+02 ; 5.251770e+03 ];
omc_error_13 = [ 5.257682e-03 ; 8.067381e-03 ; 8.358712e-03 ];
Tc_error_13  = [ 4.362303e+01 ; 3.442357e+01 ; 2.561791e+01 ];

%-- Image #14:
omc_14 = [ -1.979321e+00 ; 2.549840e-01 ; -1.217716e-02 ];
Tc_14  = [ -7.995070e+02 ; 5.642081e+02 ; 7.593624e+03 ];
omc_error_14 = [ 6.935532e-03 ; 5.433331e-03 ; 8.808237e-03 ];
Tc_error_14  = [ 6.260233e+01 ; 4.977646e+01 ; 2.501543e+01 ];

%-- Image #15:
omc_15 = [ -2.012911e+00 ; -1.275428e+00 ; -1.124056e+00 ];
Tc_15  = [ -1.191680e+03 ; 2.545007e+02 ; 4.918811e+03 ];
omc_error_15 = [ 5.380303e-03 ; 7.739671e-03 ; 9.994727e-03 ];
Tc_error_15  = [ 4.091247e+01 ; 3.224954e+01 ; 2.578875e+01 ];

%-- Image #16:
omc_16 = [ -2.311639e+00 ; 7.672977e-01 ; 6.595605e-03 ];
Tc_16  = [ -5.361250e+02 ; 2.094142e+03 ; 7.359136e+03 ];
omc_error_16 = [ 7.118479e-03 ; 4.612076e-03 ; 1.103180e-02 ];
Tc_error_16  = [ 6.126085e+01 ; 4.870979e+01 ; 2.851539e+01 ];

%-- Image #17:
omc_17 = [ 2.312174e+00 ; 3.863843e-01 ; -2.065791e-01 ];
Tc_17  = [ -1.594768e+03 ; 1.013660e+02 ; 5.120217e+03 ];
omc_error_17 = [ 6.810669e-03 ; 4.693984e-03 ; 1.025637e-02 ];
Tc_error_17  = [ 4.249235e+01 ; 3.358689e+01 ; 2.508645e+01 ];

%-- Image #18:
omc_18 = [ 2.320784e+00 ; 3.319584e-01 ; -3.965019e-01 ];
Tc_18  = [ -1.627254e+03 ; 3.180036e+02 ; 5.443239e+03 ];
omc_error_18 = [ 6.834300e-03 ; 4.790556e-03 ; 1.023265e-02 ];
Tc_error_18  = [ 4.510954e+01 ; 3.568354e+01 ; 2.542653e+01 ];

%-- Image #19:
omc_19 = [ 2.050703e+00 ; 2.783665e-01 ; -7.697381e-01 ];
Tc_19  = [ -1.747119e+03 ; 7.888779e+02 ; 5.994029e+03 ];
omc_error_19 = [ 6.844503e-03 ; 5.887116e-03 ; 8.907955e-03 ];
Tc_error_19  = [ 4.958371e+01 ; 3.936660e+01 ; 2.704047e+01 ];

