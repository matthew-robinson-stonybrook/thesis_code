clc
clear all
%cd src/dynamics/sim/results/02_27_2022/
%cd ../02_28_2022/
file = readmatrix('RK_V2.csv');
t = file(:, 1);
KE = file(:, 2);
PE = file(:, 3);
E = file(:, 4);
q1 = file(:, 5:11)
q2 = file(:, 12:18);

%Plotting Total Energy
figure
plot(t,KE,t,PE,t,E)
title("Energy")
legend('Kinetic','Potential','Total')

% Plotting Joint Positions
figure
tiledlayout(4,2)
for i=1:7
    nexttile
    plot(t,q1(:,i))
    title(strcat('Q',string(i)))
end
sgtitle('Joint Positions')

%Plotting Joint Velcoties
figure
tiledlayout(4,2)
for i=1:7
    nexttile
    plot(t,q2(:,i))
    title(strcat('Q',string(i)))
end
sgtitle('Joint Velocities')