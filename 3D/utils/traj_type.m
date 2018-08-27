function [mode, minangle, counter] = traj_type(waypoints)
W = waypoints';
N = size(W,1)-1;
dW = nan(N,3);
cosin = nan(N-1,1);
counter = 0;
d = -0.97;

for i=1:N
    dW(i,:) = W(i+1,:)-W(i,:);
end
for i=1:(N-1)
    cosin(i) = dW(i+1,:)*(dW(i,:))'/(norm(dW(i+1,:))*norm(dW(i,:)));
    counter = counter + (cosin(i)<=d);
end

if counter == 0
    mode = 'snap';
else
    mode = 'acc';
end

minangle = 180 - ceil(acos(min(cosin))*180/pi);
disp(['Most difficult maneuvre [deg]: ', num2str(minangle)]);
end
