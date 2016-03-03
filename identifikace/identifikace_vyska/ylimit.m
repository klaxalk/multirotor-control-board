function [ xDataOut, yDataOut ] = ylimit( xDataIn, yDataIn, ymin, ymax )
%A little complicated script that removes values that are not between ymin and ymax threshold from plot data 
%and replaces them with repetitive values to preserve overall point count.

removed = 0;
yDataOut = zeros(size(yDataIn));
xDataOut = zeros(size(xDataIn));
for n = 1:1:size(yDataIn)
   if ((yDataIn(n) >= ymin) && (yDataIn(n) <= ymax))
       if(removed > 0)
           for m = 1:1:removed
               if ((m <= (removed/2)) && (removed < (n-1)))
                   xDataOut(n+m-removed-1) = xDataIn(n-removed-1)+(m*0.001);
                   yDataOut(n+m-removed-1) = yDataIn(n-removed-1);
               else
                   xDataOut(n+m-removed-1) = xDataIn(n)+((m-removed-1)*0.001);
                   yDataOut(n+m-removed-1) = yDataIn(n);
               end
           end
           removed = 0;
       end
       xDataOut(n) = xDataIn(n);
       yDataOut(n) = yDataIn(n);       
   else
       removed = removed + 1;
   end    
end
end

