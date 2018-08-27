function [Xpr,Ppr,Xfl,Pfl,K,V,cndt] =...
    kalman_filter(X0,P0,F,H,R,Q,Z, sigmaN, scale,v)
% initialization
N = length(Z);
Xpr = nan(6,N);
Xpr(:,1) = X0;
Xfl = nan(6,N);

Ppr = cell(1,N);
Ppr{1} = P0;
Pfl = cell(1,N);

K = cell(1,N);
K{1} = zeros(6,3);
Xfl(:,1) = X0;
Pfl{1} = P0;

cndt = nan(3,N);
V = nan(3,N);
% Q = zeros(size(P0));
for i=2:N
    % prediction
    Xpr(:,i) = F * Xfl(:,i-1);
    Ppr{i} = F * Pfl{i-1} * (F') + Q;
    % filtration
    K{i} = Ppr{i} * (H') / (H * Ppr{i} * (H') + R);
    Xfl(:,i) = Xpr(:,i) + K{i}*(Z(:,i) - H*Xpr(:,i));
    V(:,i) = Z(:,i) - H*Xpr(:,i);
    if nargin==8
        Pfl{i} = (eye(6) - K{i}*H) * Ppr{i};
    elseif nargin==10
        cndt(:,i) = abs(v(:,i)) > scale*sigmaN;
        if max([cndt(1,i-1),cndt(2,i-1),cndt(3,i-1)])==0 &&...
                max([cndt(1,i),cndt(2,i),cndt(3,i)])==1
            Pfl{i} = P0;
        else
            Pfl{i} = (eye(6)-K{i}*H)*Ppr{i};
        end
    end
end

end




