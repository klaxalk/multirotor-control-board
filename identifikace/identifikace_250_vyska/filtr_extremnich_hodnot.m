function [ outputVektor ] = filtr_extremnich_hodnot( inputVektor )

    outputVektor = inputVektor;
    
    for i = 2:1:length(inputVektor)
        if abs(inputVektor(i)-outputVektor(i-1)) > 0.3
           outputVektor(i) = outputVektor(i-1);
        else
           outputVektor(i) = inputVektor(i); 
        end
    end
end

