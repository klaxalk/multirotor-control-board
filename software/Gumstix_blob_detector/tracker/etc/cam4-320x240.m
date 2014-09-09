% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 268.613762234588705 ; 313.149511236832893 ];

%-- Principal point:
cc = [ 156.494014264162189 ; 136.619392400929513 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.418116750142432 ; 0.191035625266476 ; -0.000838481599563 ; 0.000917824267423 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 1.604350543843115 ; 2.025963757695976 ];

%-- Principal point uncertainty:
cc_error = [ 3.287177196407752 ; 3.465686901663753 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.019655463209369 ; 0.051984598984595 ; 0.002342433516786 ; 0.001720003336805 ; 0.000000000000000 ];

%-- Image size:
nx = 320;
ny = 240;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 21;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ -2.050530e+00 ; -1.946005e+00 ; -7.777566e-02 ];
Tc_1  = [ -1.858464e+03 ; -1.333681e+03 ; 4.882633e+03 ];
omc_error_1 = [ 8.846254e-03 ; 9.660919e-03 ; 1.860245e-02 ];
Tc_error_1  = [ 6.086446e+01 ; 5.623519e+01 ; 4.362755e+01 ];

%-- Image #2:
omc_2 = [ -2.887567e+00 ; 1.926328e-02 ; 1.034514e-01 ];
Tc_2  = [ -2.416168e+03 ; 1.011922e+03 ; 5.458932e+03 ];
omc_error_2 = [ 1.343362e-02 ; 3.456905e-03 ; 2.054247e-02 ];
Tc_error_2  = [ 6.707675e+01 ; 6.189349e+01 ; 4.888976e+01 ];

%-- Image #3:
omc_3 = [ -2.500089e+00 ; -7.349759e-01 ; 8.457595e-01 ];
Tc_3  = [ -1.535472e+03 ; -5.006905e+02 ; 8.542100e+03 ];
omc_error_3 = [ 1.345755e-02 ; 6.540418e-03 ; 1.810755e-02 ];
Tc_error_3  = [ 1.047564e+02 ; 9.444687e+01 ; 5.297061e+01 ];

%-- Image #4:
omc_4 = [ -1.019104e+00 ; -2.001931e+00 ; 2.707483e-01 ];
Tc_4  = [ -8.539109e+02 ; -2.737312e+03 ; 7.058675e+03 ];
omc_error_4 = [ 8.762919e-03 ; 1.185050e-02 ; 1.423244e-02 ];
Tc_error_4  = [ 8.857893e+01 ; 7.843907e+01 ; 5.904156e+01 ];

%-- Image #5:
omc_5 = [ 2.351971e+00 ; -1.464883e-01 ; -1.341710e+00 ];
Tc_5  = [ -9.197457e+02 ; 2.101680e+03 ; 8.476433e+03 ];
omc_error_5 = [ 1.410327e-02 ; 8.386674e-03 ; 1.547841e-02 ];
Tc_error_5  = [ 1.047874e+02 ; 9.353862e+01 ; 5.971794e+01 ];

%-- Image #6:
omc_6 = [ 2.333017e+00 ; -2.214979e-01 ; -1.388309e+00 ];
Tc_6  = [ -1.017565e+03 ; 1.809527e+03 ; 8.240207e+03 ];
omc_error_6 = [ 1.426276e-02 ; 8.679670e-03 ; 1.512148e-02 ];
Tc_error_6  = [ 1.012936e+02 ; 9.104712e+01 ; 5.706033e+01 ];

%-- Image #7:
omc_7 = [ 2.314264e+00 ; -1.709048e-01 ; -1.010289e+00 ];
Tc_7  = [ -1.689986e+03 ; 1.613068e+03 ; 7.424226e+03 ];
omc_error_7 = [ 1.364242e-02 ; 7.825292e-03 ; 1.514539e-02 ];
Tc_error_7  = [ 9.122337e+01 ; 8.224625e+01 ; 5.671236e+01 ];

%-- Image #8:
omc_8 = [ -2.023556e+00 ; -8.742095e-01 ; -2.602866e-01 ];
Tc_8  = [ -2.180769e+03 ; -1.962500e+02 ; 6.529165e+03 ];
omc_error_8 = [ 1.088316e-02 ; 8.584150e-03 ; 1.466355e-02 ];
Tc_error_8  = [ 8.017696e+01 ; 7.326915e+01 ; 5.458732e+01 ];

%-- Image #9:
omc_9 = [ -2.026051e+00 ; 5.893108e-01 ; 6.801253e-01 ];
Tc_9  = [ 1.249064e+02 ; 1.530921e+03 ; 8.344669e+03 ];
omc_error_9 = [ 1.068316e-02 ; 9.978106e-03 ; 1.362437e-02 ];
Tc_error_9  = [ 1.030478e+02 ; 9.220901e+01 ; 4.545289e+01 ];

%-- Image #10:
omc_10 = [ 2.083694e+00 ; 4.248560e-01 ; 6.828999e-01 ];
Tc_10  = [ -1.485765e+03 ; -5.832923e+02 ; 5.057733e+03 ];
omc_error_10 = [ 1.183604e-02 ; 8.720034e-03 ; 1.485751e-02 ];
Tc_error_10  = [ 6.260468e+01 ; 5.705250e+01 ; 5.108305e+01 ];

%-- Image #11:
omc_11 = [ 2.224418e+00 ; 4.134102e-01 ; 5.873210e-01 ];
Tc_11  = [ -1.719135e+03 ; -3.148634e+02 ; 5.070732e+03 ];
omc_error_11 = [ 1.197686e-02 ; 7.890175e-03 ; 1.599940e-02 ];
Tc_error_11  = [ 6.282469e+01 ; 5.731194e+01 ; 5.151143e+01 ];

%-- Image #12:
omc_12 = [ -2.724838e+00 ; 2.463095e-01 ; -1.065804e+00 ];
Tc_12  = [ 6.401249e+01 ; 1.003041e+03 ; 5.566403e+03 ];
omc_error_12 = [ 1.417390e-02 ; 7.387612e-03 ; 1.834644e-02 ];
Tc_error_12  = [ 6.824848e+01 ; 6.143973e+01 ; 4.623806e+01 ];

%-- Image #13:
omc_13 = [ -2.771553e+00 ; 8.749726e-02 ; 1.127523e+00 ];
Tc_13  = [ -6.799709e+02 ; 9.680683e+02 ; 7.772192e+03 ];
omc_error_13 = [ 1.304121e-02 ; 7.141924e-03 ; 1.818522e-02 ];
Tc_error_13  = [ 9.473965e+01 ; 8.566420e+01 ; 4.461842e+01 ];

%-- Image #14:
omc_14 = [ -1.923912e+00 ; 1.820127e-01 ; 1.274025e-01 ];
Tc_14  = [ -1.119139e+03 ; 3.619704e+01 ; 7.330792e+03 ];
omc_error_14 = [ 1.104279e-02 ; 8.154395e-03 ; 1.276305e-02 ];
Tc_error_14  = [ 8.957030e+01 ; 8.125321e+01 ; 3.999212e+01 ];

%-- Image #15:
omc_15 = [ 2.094744e+00 ; -2.001545e-01 ; 1.266323e-01 ];
Tc_15  = [ -1.166638e+03 ; 3.348559e+02 ; 4.039357e+03 ];
omc_error_15 = [ 1.181499e-02 ; 8.490857e-03 ; 1.344823e-02 ];
Tc_error_15  = [ 4.974403e+01 ; 4.512495e+01 ; 3.933549e+01 ];

%-- Image #16:
omc_16 = [ -2.776681e+00 ; 1.968715e-01 ; 7.231380e-02 ];
Tc_16  = [ -4.768724e+02 ; 1.821465e+03 ; 7.256417e+03 ];
omc_error_16 = [ 1.388966e-02 ; 4.462793e-03 ; 2.266213e-02 ];
Tc_error_16  = [ 8.939829e+01 ; 7.902833e+01 ; 5.114735e+01 ];

%-- Image #17:
omc_17 = [ -2.735036e+00 ; -1.369033e-02 ; 9.955197e-02 ];
Tc_17  = [ -1.121300e+03 ; 1.557697e+03 ; 6.876119e+03 ];
omc_error_17 = [ 1.281087e-02 ; 4.476965e-03 ; 2.045602e-02 ];
Tc_error_17  = [ 8.443419e+01 ; 7.510402e+01 ; 4.801317e+01 ];

%-- Image #18:
omc_18 = [ -2.735146e+00 ; -5.238013e-02 ; 1.098149e-01 ];
Tc_18  = [ -2.391947e+03 ; 1.453290e+03 ; 6.665108e+03 ];
omc_error_18 = [ 1.285146e-02 ; 4.798319e-03 ; 2.069066e-02 ];
Tc_error_18  = [ 8.229745e+01 ; 7.504632e+01 ; 5.286519e+01 ];

%-- Image #19:
omc_19 = [ -2.786737e+00 ; 1.249179e-01 ; 1.438899e-01 ];
Tc_19  = [ 6.236530e+02 ; 1.612039e+03 ; 6.679878e+03 ];
omc_error_19 = [ 1.409888e-02 ; 4.619903e-03 ; 2.385966e-02 ];
Tc_error_19  = [ 8.368609e+01 ; 7.270410e+01 ; 4.709256e+01 ];

%-- Image #20:
omc_20 = [ -2.177669e+00 ; 5.315801e-02 ; 3.109539e-01 ];
Tc_20  = [ -9.401829e+02 ; 1.337168e+03 ; 7.127610e+03 ];
omc_error_20 = [ 1.089460e-02 ; 7.813125e-03 ; 1.415487e-02 ];
Tc_error_20  = [ 8.798568e+01 ; 7.831591e+01 ; 3.833771e+01 ];

%-- Image #21:
omc_21 = [ 2.185707e+00 ; -1.936901e-02 ; -2.180184e-01 ];
Tc_21  = [ -1.591293e+03 ; 7.266241e+02 ; 4.061664e+03 ];
omc_error_21 = [ 1.201537e-02 ; 7.517827e-03 ; 1.413741e-02 ];
Tc_error_21  = [ 5.003608e+01 ; 4.567264e+01 ; 3.988240e+01 ];

