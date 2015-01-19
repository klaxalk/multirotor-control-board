% A_roof matrix
dlmwrite('A_roof.txt', A_roof, 'precision', '%15.10f');

% diagonal of Q_roof matrix
dlmwrite('Q_roof_diag.txt', diag(Q_roof), 'precision', '%3.1f');

% B_roof matrix
dlmwrite('B_roof.txt', B_roof, 'precision', '%15.10f');

% H_inv matrix
dlmwrite('H_inv.txt', H_inv, 'precision', '%20.10f');