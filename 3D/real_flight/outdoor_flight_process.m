filename = 'outdoor_flight_vehicle_local_position_0.csv';
matrix = csvread(filename);
[~, cols] = size(matrix);
data = cell(cols,1);
for i=1:cols
    data{i} = matrix(:,i);
end
x = data{5};
y = data{6};
z = data{7};
figure
plot(matrix(:,5));
hold on
plot(matrix(:,6));
hold on
plot(matrix(:,7));
grid on
legend('x','y','z');


