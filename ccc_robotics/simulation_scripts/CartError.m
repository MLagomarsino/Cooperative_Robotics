function [temp1 , temp2] = CartError(in1, in2)
% computes the Cartesian error between two frames
% e.g. CartError(wTg, wTv) returns the error that makes <v> -> <g> 
% projected on <w>

% ---- 
% Paramters:
% - in1, in2 : two trasformation matrices projected on the same frame
% Return values:
% - temp1 : - rotation vector between the two frames
% - temp2 : basic vector

temp1 = (VersorLemma(in1(1:3,1:3), in2(1:3,1:3)))*(-1);
temp2 = in1(1:3,4)-in2(1:3,4);

end

