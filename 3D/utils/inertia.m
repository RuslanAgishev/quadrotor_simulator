function I = inertia(M,m,R,l)
Ix = 2/5*M*(R^2) + 2*m*(l^2);
Iy = Ix;
Iz = 2/5*M*(R^2) + 4*m*(l^2);
I = zeros(3);
I(1,1) = Ix;
I(2,2) = Iy;
I(3,3) = Iz;
end