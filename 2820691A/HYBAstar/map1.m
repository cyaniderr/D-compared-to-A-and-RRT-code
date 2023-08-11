% preamble 
clearvars
close all
clc;

x_max = 100;
y_max = 100;



Mapmatrix = zeros(x_max,y_max);

obs = {[5,70,20,20];[40,70,20,20];[75,70,20,20];[5,40,20,20];[40,40,20,20];[75,40,20,20];[5,10,20,20];[40,10,20,20];[75,10,20,20]};

for k = 1:length(obs)

    for i = obs{k}(1):(obs{k}(1)+obs{k}(3))
        for j = obs{k}(2):(obs{k}(2)+obs{k}(4))
            Mapmatrix(i,j) = 1;
        end
    end
end

flipper = flip(Mapmatrix);
Mapmatrixx = flipper;
mymap = binaryOccupancyMap(Mapmatrixx);
% show(mymap);
% axis([1, x_max, 1 ,y_max]);