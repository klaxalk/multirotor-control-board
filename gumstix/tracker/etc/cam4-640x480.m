% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 537.066607941750021 ; 629.460589654974569 ];

%-- Principal point:
cc = [ 313.296878626129683 ; 270.839909304211517 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.414257281883143 ; 0.194822431043693 ; 0.001417306324019 ; 0.000650040797294 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 0.589962230978555 ; 0.684911417469102 ];

%-- Principal point uncertainty:
cc_error = [ 1.126802722278375 ; 1.160221370864403 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.003444256694995 ; 0.009615056154670 ; 0.000405379587992 ; 0.000308103173015 ; 0.000000000000000 ];

%-- Image size:
nx = 640;
ny = 480;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 20;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ -3.035457e+00 ; -4.796801e-02 ; 1.286666e-01 ];
Tc_1  = [ -1.585235e+03 ; 1.129731e+03 ; 5.575630e+03 ];
omc_error_1 = [ 2.608364e-03 ; 5.403022e-04 ; 3.762121e-03 ];
Tc_error_1  = [ 1.178537e+01 ; 1.027850e+01 ; 8.138361e+00 ];

%-- Image #2:
omc_2 = [ -3.103295e+00 ; 1.759158e-02 ; 1.863845e-01 ];
Tc_2  = [ -1.585398e+03 ; 1.141341e+03 ; 5.342901e+03 ];
omc_error_2 = [ 2.494306e-03 ; 5.294567e-04 ; 3.743621e-03 ];
Tc_error_2  = [ 1.129982e+01 ; 9.905557e+00 ; 7.903909e+00 ];

%-- Image #3:
omc_3 = [ -3.091114e+00 ; 2.889111e-02 ; 2.010638e-01 ];
Tc_3  = [ -1.015141e+03 ; 1.140863e+03 ; 5.133474e+03 ];
omc_error_3 = [ 2.420901e-03 ; 5.410263e-04 ; 3.736222e-03 ];
Tc_error_3  = [ 1.087848e+01 ; 9.440837e+00 ; 7.353818e+00 ];

%-- Image #4:
omc_4 = [ 2.995211e+00 ; 5.601687e-02 ; -8.848404e-02 ];
Tc_4  = [ -2.618369e+03 ; 8.779931e+02 ; 4.879106e+03 ];
omc_error_4 = [ 2.281068e-03 ; 6.312135e-04 ; 3.820200e-03 ];
Tc_error_4  = [ 1.037925e+01 ; 9.441939e+00 ; 9.128369e+00 ];

%-- Image #5:
omc_5 = [ -2.144341e+00 ; 2.652918e-02 ; 2.096256e-01 ];
Tc_5  = [ -2.060157e+03 ; 6.160352e+02 ; 6.698172e+03 ];
omc_error_5 = [ 1.930650e-03 ; 1.300886e-03 ; 2.405714e-03 ];
Tc_error_5  = [ 1.405457e+01 ; 1.257085e+01 ; 7.620570e+00 ];

%-- Image #6:
omc_6 = [ -1.969859e+00 ; 6.420658e-02 ; 1.263630e-01 ];
Tc_6  = [ -1.703012e+03 ; -1.672021e+02 ; 6.798434e+03 ];
omc_error_6 = [ 1.914107e-03 ; 1.321948e-03 ; 2.233515e-03 ];
Tc_error_6  = [ 1.422536e+01 ; 1.269715e+01 ; 7.049335e+00 ];

%-- Image #7:
omc_7 = [ 1.931651e+00 ; 1.024762e-03 ; -2.440046e-01 ];
Tc_7  = [ -2.005365e+03 ; 1.158293e+03 ; 5.071245e+03 ];
omc_error_7 = [ 1.944908e-03 ; 1.412383e-03 ; 2.230055e-03 ];
Tc_error_7  = [ 1.078251e+01 ; 9.618337e+00 ; 8.367277e+00 ];

%-- Image #8:
omc_8 = [ 2.076874e+00 ; 5.703924e-03 ; -3.347584e-01 ];
Tc_8  = [ -1.937606e+03 ; 1.392899e+03 ; 5.095948e+03 ];
omc_error_8 = [ 1.968357e-03 ; 1.280387e-03 ; 2.385172e-03 ];
Tc_error_8  = [ 1.087759e+01 ; 9.652820e+00 ; 8.219153e+00 ];

%-- Image #9:
omc_9 = [ 1.733255e+00 ; -1.657856e+00 ; -7.930881e-01 ];
Tc_9  = [ 1.502160e+03 ; 1.059383e+03 ; 5.010319e+03 ];
omc_error_9 = [ 1.921261e-03 ; 1.668061e-03 ; 2.646538e-03 ];
Tc_error_9  = [ 1.082433e+01 ; 9.403481e+00 ; 8.530334e+00 ];

%-- Image #10:
omc_10 = [ -2.969395e+00 ; 6.264293e-02 ; -1.019399e+00 ];
Tc_10  = [ -1.514123e+03 ; 1.242809e+03 ; 5.494826e+03 ];
omc_error_10 = [ 2.395178e-03 ; 1.132411e-03 ; 3.482524e-03 ];
Tc_error_10  = [ 1.190064e+01 ; 1.028357e+01 ; 9.051623e+00 ];

%-- Image #11:
omc_11 = [ 2.809759e+00 ; 1.563864e-02 ; 1.257212e+00 ];
Tc_11  = [ -9.821529e+02 ; 9.638745e+02 ; 4.768596e+03 ];
omc_error_11 = [ 2.291354e-03 ; 1.347447e-03 ; 3.155169e-03 ];
Tc_error_11  = [ 1.021555e+01 ; 8.852873e+00 ; 7.803761e+00 ];

%-- Image #12:
omc_12 = [ -2.212002e+00 ; -1.419827e-01 ; -8.608595e-01 ];
Tc_12  = [ -7.187418e+02 ; 1.158802e+03 ; 5.418316e+03 ];
omc_error_12 = [ 2.001706e-03 ; 1.485565e-03 ; 2.492768e-03 ];
Tc_error_12  = [ 1.143550e+01 ; 9.890200e+00 ; 7.274054e+00 ];

%-- Image #13:
omc_13 = [ -2.213899e+00 ; -6.377653e-02 ; -8.665706e-01 ];
Tc_13  = [ -7.491490e+02 ; 8.359997e+02 ; 5.340735e+03 ];
omc_error_13 = [ 2.011132e-03 ; 1.401808e-03 ; 2.504678e-03 ];
Tc_error_13  = [ 1.119505e+01 ; 9.772375e+00 ; 6.961579e+00 ];

%-- Image #14:
omc_14 = [ 2.232204e+00 ; 2.114540e-01 ; 8.527342e-01 ];
Tc_14  = [ -3.904481e+02 ; -4.776968e+01 ; 4.599215e+03 ];
omc_error_14 = [ 2.103332e-03 ; 1.469309e-03 ; 2.530009e-03 ];
Tc_error_14  = [ 9.779799e+00 ; 8.527553e+00 ; 7.987299e+00 ];

%-- Image #15:
omc_15 = [ 2.261221e+00 ; 1.830205e-01 ; 7.889926e-01 ];
Tc_15  = [ -5.374000e+02 ; 1.120771e+02 ; 4.710032e+03 ];
omc_error_15 = [ 2.094649e-03 ; 1.415269e-03 ; 2.583139e-03 ];
Tc_error_15  = [ 1.000803e+01 ; 8.734434e+00 ; 8.160388e+00 ];

%-- Image #16:
omc_16 = [ 2.122736e+00 ; -1.244737e-01 ; -1.303369e+00 ];
Tc_16  = [ 5.446731e+02 ; 1.578743e+03 ; 7.285719e+03 ];
omc_error_16 = [ 2.335044e-03 ; 1.496262e-03 ; 2.419033e-03 ];
Tc_error_16  = [ 1.552694e+01 ; 1.337190e+01 ; 8.671729e+00 ];

%-- Image #17:
omc_17 = [ 2.212738e+00 ; -1.303583e+00 ; -1.150959e+00 ];
Tc_17  = [ 1.059334e+03 ; 1.485430e+03 ; 6.431965e+03 ];
omc_error_17 = [ 2.345084e-03 ; 1.333772e-03 ; 2.961792e-03 ];
Tc_error_17  = [ 1.357273e+01 ; 1.193170e+01 ; 9.458164e+00 ];

%-- Image #18:
omc_18 = [ -3.136113e+00 ; 8.552195e-02 ; -2.106188e-02 ];
Tc_18  = [ 3.064250e+02 ; 1.774050e+03 ; 7.166660e+03 ];
omc_error_18 = [ 3.105649e-03 ; 8.471323e-04 ; 5.870978e-03 ];
Tc_error_18  = [ 1.534644e+01 ; 1.328650e+01 ; 1.150521e+01 ];

%-- Image #19:
omc_19 = [ 3.129549e+00 ; -9.235095e-02 ; -1.093426e-01 ];
Tc_19  = [ 1.263624e+02 ; 1.445121e+03 ; 6.284254e+03 ];
omc_error_19 = [ 2.724134e-03 ; 8.104438e-04 ; 5.312060e-03 ];
Tc_error_19  = [ 1.341496e+01 ; 1.166104e+01 ; 9.915795e+00 ];

%-- Image #20:
omc_20 = [ 3.115627e+00 ; -1.193960e-01 ; -2.574386e-01 ];
Tc_20  = [ -2.371472e+03 ; 1.654533e+03 ; 6.404443e+03 ];
omc_error_20 = [ 2.490164e-03 ; 7.805091e-04 ; 4.750273e-03 ];
Tc_error_20  = [ 1.367738e+01 ; 1.206675e+01 ; 1.014455e+01 ];

