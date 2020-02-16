function pl = calcCoilsPowerLoss(current, resistance, frequency, parameters)
% CALCPL            Calculates power loss in the coils
    pl = current^2 * resistance;
end