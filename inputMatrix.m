function U_Matrix = inputMatrix(w_RotorSpeed, K_f, K_m, l_armLength)
%INPUTMATRIX Summary of this function goes here
%   Detailed explanation goes here
U1 = K_f * (norm(w_RotorSpeed,2)^2); % NOTE: norm = sqrt( w1^2 + w2^2 + ...)
U2 = l_armLength* K_f * (w_RotorSpeed(4)^2 - (w_RotorSpeed(2))^2 );
U3 = l_armLength* K_f * (w_RotorSpeed(3)^2 - (w_RotorSpeed(1))^2 );
U4 = K_m * (w_RotorSpeed(2)^2 + (w_RotorSpeed(4))^2 ...
    - (w_RotorSpeed(1))^2 - (w_RotorSpeed(3))^2);
U_Matrix = [U1 U2 U3 U4]';
end

