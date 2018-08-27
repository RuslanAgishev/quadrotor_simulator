function [F,G,H] = state_space(T)
F = eye(6);
F(1,2) = T;
F(3,4) = T;
F(5,6) = T;
G = [T^2/2 0 0; T 0 0; 0 T^2/2 0; 0 T 0; 0 0 T^2/2; 0 0 T];
H = zeros(3,6);
H(1,1) = 1;
H(2,3) = 1;
H(3,5) = 1;
end