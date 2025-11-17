% Define the system matrix A
A = [0 0 1 0 0; 
     0 0 0 0 0; 
     gg/hh (V^2/bike_params.h-(gg*bike_params.lr*bike_params.c/(bike_params.h^2*(bike_params.lr+bike_params.lf))))*sin(bike_params.lambda) 0 0 0; 
     0 0 0 -lr * V * k1 / (lr + lf) V - lr * V * k2 / (lr + lf); 
     0 0 0 -V * k1 / (lr + lf) -V * k2 / (lr + lf)];

% Compute the eigenvalues (characteristic roots) of matrix A
eigenvalues = eig(A);

% Display the eigenvalues
disp('Eigenvalues of the system matrix A:');
disp(eigenvalues);

