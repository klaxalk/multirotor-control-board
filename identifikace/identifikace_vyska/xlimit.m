function [ xDataOut, yDataOut ] = xlimit( xDataIn, yDataIn, xmin, xmax)
%A simple script that limits input plot data to match desired x limits. 

position = 0;
yDataOut = zeros(size(yDataIn));
xDataOut = zeros(size(xDataIn));
for n = 1:1:size(yDataIn)
   if ((xDataIn(n) <= xmax) && (xDataIn(n) >= xmin))
       position = position + 1;
       yDataOut(position) = yDataIn(n);
       xDataOut(position) = xDataIn(n);
   end    
end
yDataOut = yDataOut(1:position);
xDataOut = xDataOut(1:position);
end

