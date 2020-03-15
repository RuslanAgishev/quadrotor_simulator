close all
clear all

pc = pcread('../data/fountain.ply');
pcshow(pc);

points = pc.Location;

Z = points(:,3);
Z = Z(Z>0);