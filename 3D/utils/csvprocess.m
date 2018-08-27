function data = csvprocess(filename)
matrix = csvread(filename);
[rows, cols] = size(matrix);
data = cell(cols,1);
for i=1:cols
    data{i} = matrix(:,i);
end
x = data{6};
y = data{7};
z = data{8};
figure
plot(matrix(:,6));
hold on
plot(matrix(:,7));
hold on
plot(matrix(:,8));
grid on
legend('x','y','z');
end