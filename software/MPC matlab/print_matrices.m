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

fid = fopen('mpcMatrices.c', 'w');

fprintf(fid, '#include "CMatrixLib.h"\n');
fprintf(fid, '#include "mpcMatrices.h"\n\n\n');

%% Print A_roof

printMatrix(fid, 'const float A_roof_data[A_ROOF_HEIGHT*A_ROOF_WIDTH]', '%15.10f', A_roof);

%% Print Q_roof_diag

printMatrix(fid, 'const float Q_roof_diag_data[Q_ROOF_DIAG_SIZE]', '%3.1f', diag(Q_roof));

%% Print B_roof

printMatrix(fid, 'const float B_roof_data[B_ROOF_HEIGHT*B_ROOF_WIDTH]', '%15.10f', B_roof);

%% Print H_inv

printMatrix(fid, 'const float H_inv_data[H_INV_HEIGHT*H_INV_WIDTH]', '%20.10f', H_inv);