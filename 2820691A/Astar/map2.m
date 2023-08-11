% preamble 
clearvars
close all
clc;

x_max = 100;
y_max = 100;



Mapmatrix = zeros(x_max,y_max);

% obs = {[30,30,40,40]};
% 
% for k = 1:length(obs)
% 
%     for i = obs{k}(1):(obs{k}(1)+obs{k}(3))
%         for j = obs{k}(2):(obs{k}(2)+obs{k}(4))
%             Mapmatrix(i,j) = 1;
%         end
%     end
% end

flipper = flip(Mapmatrix);
Mapmatrixx = flipper;
mymap = binaryOccupancyMap(Mapmatrixx);
% show(mymap);hold on;title('');grid on
% xlabel('Y(meters)'); ylabel('X(meters)');
% plot(2, 2, 'g*') 
% plot(95, 95, 'b*')
% axis([1, x_max, 1 ,y_max]);