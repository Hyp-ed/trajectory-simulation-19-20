function [ parameters ] = loadParameters()
%  importLimParameters Imports the Lim parameters
%  Inputs:
%  Output: 
%    [ parameters ]      Parameter structure containing imported variables
%  @author              Rafael Anderka 10/01/2020

    fname = './parameters.json';
    parameters = jsondecode(fileread(fname));
    
end