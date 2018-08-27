function r = runningmean(z,M)
% obtaining accurate measurements from noisy ones 
% using Running mean-method: r = 1/M sum(z)
r = zeros(1,length(z));
if rem(M,2)==0
    M = M-1;
end
r(1:(M-1)/2) = mean(z(1:(M-1)/2));
r(length(z)-(M-1)/2+1 : length(z)) = mean(z(length(z)-(M-1)/2+1 : length(z)));
for i=((M-1)/2+1) : (length(z)-(M-1)/2)
    r(i) = 1/M*sum(z(i-(M-1)/2 : i+(M-1)/2));
end
end