%% Generates a 2D lookup table that maps velocity to optimal slip
%  @author Rafael Anderka, HYPED 2019
%  modified: Lorenzo Principe Hyped 2019/20
clc; clear; close all;

%% Params
fx_lookup_table = load('forceLookupTable.mat');
v_min = fx_lookup_table.velocities(1);
v_max = fx_lookup_table.velocities(end);
v_step = fx_lookup_table.v_step;
s_min = fx_lookup_table.slips(1);
s_max = fx_lookup_table.slips(end);
s_step = fx_lookup_table.s_step;

%% Get optimal values
optimalSlip = zeros(1,length(fx_lookup_table.velocities));
optimalRPM = zeros(1,length(fx_lookup_table.velocities));

for i = 1:length(fx_lookup_table.velocities)
    [~, optimalSlip(i)] = max(fx_lookup_table.smoothForces(i,:));
    optimalSlip(i) = s_step * (optimalSlip(i)-1);
    
    optimalRPM(i) = ((optimalSlip(i) + fx_lookup_table.velocities(i)) / r) * 60 / (2*pi);
end

%% Plot
figure(1);
hold on;
plot(fx_lookup_table.velocities,optimalRPM)
xlabel('Velocity (m/s)')
ylabel('RPM')

optimalRPMCoefficients = polyfit(fx_lookup_table.velocities,optimalRPM,2);
y = polyval(optimalRPMCoefficients,fx_lookup_table.velocities);
plot(fx_lookup_table.velocities,y);
fprintf('%.10f v^2 + %.10f v + %.10f\n',optimalRPMCoefficients(1),optimalRPMCoefficients(2),optimalRPMCoefficients(3));

%{
figure(2);
hold on;
plot(fx_lookup_table.velocities,optimalSlip)
xlabel('Velocity (m/s)')
ylabel('Slip (m/s)')

optimalSlipCoefficients = polyfit(fx_lookup_table.velocities,optimalSlip,1);
y = polyval(optimalSlipCoefficients,fx_lookup_table.velocities);
plot(fx_lookup_table.velocities,y);

save('optimalSlipCoefficients.mat',"optimalSlipCoefficients");
%}
