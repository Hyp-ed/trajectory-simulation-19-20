clear

data = xlsread('Parameters\19-11-17_LIMForceTable_Export.xlsx');
 

freq_step   = 5
v_step      = 2

freq_max    = 800
v_max       = 100

frequencies = 0:freq_step:freq_max
velocities  = 0:v_step:v_max
forces      = data(1:end,3)

forces = reshape(forces,length(velocities),length(frequencies))



save('forceLookupTable.mat')

