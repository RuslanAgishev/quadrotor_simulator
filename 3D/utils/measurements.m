function [xm,ym,zm] = measurements(state, sigmaN)
x = state(:,1);
y = state(:,2);
z = state(:,3);

xm = nan(size(x));
ym = nan(size(y));
zm = nan(size(z));
N = length(state);
for i=1:N
   nx = randn(1)*sigmaN;
   ny = randn(1)*sigmaN;
   nz = randn(1)*sigmaN;
   xm(i) = x(i) + nx;
   ym(i) = y(i) + ny;
   zm(i) = z(i) + nz;   
end

end


