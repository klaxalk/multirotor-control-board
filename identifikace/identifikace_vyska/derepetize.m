function [ xDataOut, yDataOut ] = derepetize( xDataIn, yDataIn )
%A simple script that removes repetitive values in Y coordinate from plot data. 

position = 0;
yDataOut = zeros(size(yDataIn));
xDataOut = zeros(size(xDataIn));
yDataOut(1) = yDataIn(1);
xDataOut(1) = xDataIn(1);
for n = 2:1:size(yDataIn)
   if (yDataIn(n) ~= yDataIn(n-1))
       position = position + 1;
       yDataOut(position) = yDataIn(n);
       xDataOut(position) = xDataIn(n);
   end    
end
yDataOut = yDataOut(1:position);
xDataOut = xDataOut(1:position);
end

