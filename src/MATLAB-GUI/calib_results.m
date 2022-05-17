clc;
clear all;
load results_calib_2.mat
x = 0:0.01:1.56;
figure;
%subplot(3,1,1)
plot(x,save_results(:,2))
hold on
plot(x,save_results(:,4))
a = mean(abs(save_results(:,2)));
b = mean(abs(save_results(:,4)));
disp('Mean GT=')
disp(a)
disp('Mean Sensor =')
disp(b)
title('X Error of Infra Camera')
%calculate variance
Cov_x = cov(save_results(:,4));
mean_diff_x = (a-b)^2;
total_Cov_x = sqrt(Cov_x + mean_diff_x); 
disp('Cov_x =')
disp(total_Cov_x)
figure;
%subplot(3,1,2)
plot(x,save_results(:,3))
hold on
plot(x,save_results(:,5))
c = mean(abs(save_results(:,3)));
d = mean(abs(save_results(:,5)));
disp('Mean GT=')
disp(c)
disp('Mean Sensor =')
disp(d)
title('Y Error of Infra Camera')
%calculate variance
Cov_y = cov(save_results(:,5));
mean_diff_y = (c-d)^2;
total_Cov_y = sqrt(Cov_y + mean_diff_y); 
disp('Cov_y =')

disp(total_Cov_y)
hold on;
disp(Cov_y)
disp(Cov_x)