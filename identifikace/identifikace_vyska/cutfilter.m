function [ xDataOut, yDataOut ] = cutfilter( xDataIn, yDataIn, maxdif, mindif)
%A simple script that removes sudden changes in Y from the data based on the sum of differences. 

position = 1;
yDataOut = zeros(size(yDataIn));
xDataOut = zeros(size(xDataIn));
yDataOut(1) = yDataIn(1);
xDataOut(1) = xDataIn(1);
for n = 2:1:(size(yDataIn)-1)
   if ((((yDataIn(n) - yDataIn(n-1)) - (yDataIn(n+1) - yDataIn(n))) <= maxdif) && (((yDataIn(n) - yDataIn(n-1)) - (yDataIn(n+1) - yDataIn(n))) >= mindif))
       position = position + 1;
       yDataOut(position) = yDataIn(n);
       xDataOut(position) = xDataIn(n);
   else
      yDataIn(n) = (yDataIn(n-1) + yDataIn(n+1)) / 2;
   end    
end
yDataOut = yDataOut(1:position);
xDataOut = xDataOut(1:position);
end