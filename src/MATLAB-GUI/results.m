clc;
clear all;
load results.mat
x = 0:0.01:0.41;
figure;
%subplot(3,1,1)
plot(x,save_results(:,1))
a = mean(abs(save_results(:,1)));
b = max(abs(save_results(:,1)));
c = min(abs(save_results(:,1)));
disp('MSE =')
disp(a)
disp('Max Error =')
disp(b)
disp('Min Error =')
disp(c)
xlabel('time(s)')
ylabel('Position-error(m)')
title('Position Error of EKF')

figure;
%subplot(3,1,2)
plot(x,save_results(:,2))
xlabel('time(s)')
ylabel('Orientation-error(rad)')
title('Orientation Error')
d = mean(abs(save_results(:,2)));
e = max(abs(save_results(:,2)));
f = min(abs(save_results(:,2)));
disp('MSE =')
disp(d)
disp('Max Error =')
disp(e)
disp('Min Error =')
disp(f)

figure;
%subplot(3,1,3)
plot(save_results(:,3),save_results(:,4));
xlabel('x')
ylabel('y')
title('trajectory');

hold on;