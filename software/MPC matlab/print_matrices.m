% % A_roof matrix
% dlmwrite('A_roof.txt', A_roof, 'precision', '%15.10f');
% 
% % diagonal of Q_roof matrix
% dlmwrite('Q_roof_diag.txt', diag(Q_roof), 'precision', '%3.1f');
% 
% % B_roof matrix
% dlmwrite('B_roof.txt', B_roof, 'precision', '%15.10f');
% 
% % H_inv matrix
% dlmwrite('H_inv.txt', H_inv, 'precision', '%20.10f');

fid = fopen('elevAileMpcMatrices.c', 'w');

fprintf(fid, '#include "CMatrixLib.h"\n');
fprintf(fid, '#include "elevAileMpcMatrices.h"\n\n');

fprintf(fid, '/*\n');
fprintf(fid, 'This file was created automatically with following parameters\n\n');

printMatrixM(fid, 'dt', '%1.4f', dt);
printMatrixM(fid, 'A', '%1.8f', A);
printMatrixM(fid, 'B', '%1.8f', B);
printMatrixM(fid, 'horizon_len', '%d', horizon_len);
printMatrixM(fid, 'n_variables', '%d', n_variables);
printMatrixM(fid, 'Q', '%1.1f', Q);
printMatrixM(fid, 'S', '%1.1f', S);
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