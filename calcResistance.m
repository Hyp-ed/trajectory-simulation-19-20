function resistance = calcResistance(parameters, coilsTemp)
    resistivity = parameters.coilsResistivity * (1 + (coilsTemp - parameters.refTemp));
    resistance = resistivity * parameters.wireLength / parameters.wireArea;
end