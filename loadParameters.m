function [ parameters ] = loadParameters()
%  importLimParameters Imports the Lim parameters
%  Inputs:
%  Output: 
%    [ parameters ]      Parameter structure containing imported variables
%  @author              Rafael Anderka 02/11/2018

    fname = './config/config.json';
    parameters = jsondecode(fileread(fname));
    
end