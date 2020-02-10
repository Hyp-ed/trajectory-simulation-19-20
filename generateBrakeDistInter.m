function inter = generateBrakeDistInter(param,brakeTable)
%GENERATEBRAKEDISTINTER Summary of this function goes here
%   Detailed explanation goes here

display('- generating braking distance interpolator (assuming max speed = 50m/s)')

maxV = 50;
vSteps = 1000;
dt = param.dt;

velList = linspace(0,maxV,vSteps);
dist = zeros(1,length(velList)); 

for i = 1:length(velList)
   v = velList(i);
   x = 0;
   while v > 0
       brakingForce = calcBrakingForce(v,param);
       acceleration = brakingForce / param.mass;
       v = v + acceleration*dt;
       x = x + v*dt;
   end
   
   dist(i) = x;
end

inter = griddedInterpolant(velList,dist);

end

