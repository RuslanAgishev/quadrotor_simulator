function vmean = mean_velocity(v)
vx = v(:,1);
vy = v(:,2);
vz = v(:,3);
N = length(v);
V = zeros(1,N);
for i=1:N
    V(i) = sqrt((vx(i))^2 + (vy(i))^2 + (vz(i))^2);
end
vmean = sum(V)/N;

end