clc
clear
close all
%% 地图构建
map=imread('map2.png');
map=rgb2gray(map);
map=imbinarize(map);
y_num=size(map,1);
x_num=size(map,2);
map1=map(40:y_num-80,40:x_num-80);
map=map1(round(1:size(map1,1)/200:size(map1,1)),round(1:size(map1,2)/200:size(map1,2)));
% map=flipud(map);
imshow(map)
save('map.mat','map')