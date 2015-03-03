function [ out ] = diference( in )

    out(1) = 0;
    
    for i=2:length(in)
       out(i) = in(i) - in(i-1); 
    end

end

