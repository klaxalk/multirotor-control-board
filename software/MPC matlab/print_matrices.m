% This script creates ANSI C code with pregenerated matrices for MPC. It
% should be run after tunning and simulation by "main.m" which initializes
% these matrices.

fid = fopen('elevAileMpcMatrices.c', 'w');

fprintf(fid, '#include "CMatrixLib.h"\n');
fprintf(fid, '#include "elevAileMpcMatrices.h"\n\n');

% print comment which identifies these matrices (model?)
fprintf(fid, '// tricopter, KK2, stick scaling = 10\n\n');

fprintf(fid, '/*\n');
fprintf(fid, 'This file was created automatically with following parameters\n\n');

printMatrixM(fid, 'dt', '%1.4f', dt);
printMatrixM(fid, 'A', '%1.8f', A);
printMatrixM(fid, 'B', '%1.8f', B);
printMatrixM(fid, 'horizon_len', '%d', horizon_len);
printMatrixM(fid, 'n_variables', '%d', n_variables);
printMatrixM(fid, 'Q', '%1.3f', Q);
printMatrixM(fid, 'S', '%1.3f', S);
printMatrixM(fid, 'P', '%1.10f', P);

fprintf(fid, '*/\n\n');

%% Print A_roof

printMatrixC(fid, 'const float A_roof_data_Attitude[ATTITUDE_A_ROOF_HEIGHT*ATTITUDE_A_ROOF_WIDTH]', '%15.20f', A_roof);

%% Print Q_roof_diag

printMatrixC(fid, 'const float Q_roof_diag_data_Attitude[ATTITUDE_Q_ROOF_DIAG_SIZE]', '%3.1f', diag(Q_roof));

%% Print B_roof

printMatrixC(fid, 'const float B_roof_data_Attitude[ATTITUDE_B_ROOF_HEIGHT*ATTITUDE_B_ROOF_WIDTH]', '%15.20f', B_roof);

%% Print H_inv

printMatrixC(fid, 'const float H_inv_data_Attitude[ATTITUDE_H_INV_HEIGHT*ATTITUDE_H_INV_WIDTH]', '%15.20f', H_inv);