% LENGTH calculates the length of the trajectory of a quadrotor
% x, y, z - vectors containing consequentive coordinates of the UAV
% length(x) = length(y) = length(z) = N
function L = length_traj(r)
x = r(:,1);
y = r(:,2);
z = r(:,3);
N = size(r,1);
d = zeros(1,N-1);
for i=1:(N-1)
   d(i) = sqrt((x(i+1)-x(i))^2 + (y(i+1)-y(i))^2 + (z(i+1)-z(i))^2);
end
L = sum(d);
end