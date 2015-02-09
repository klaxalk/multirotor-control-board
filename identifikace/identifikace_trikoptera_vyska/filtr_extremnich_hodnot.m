function [ outputVektor ] = filtr_extremnich_hodnot( inputVektor )

    outputVektor = inputVektor;
    
    smerOdchylka = std(inputVektor);
    prumer = mean(inputVektor);
    
    for i = 2:1:length(inputVektor)
        if abs(inputVektor(i)-prumer) > smerOdchylka*0.6
           outputVektor(i) = outputVektor(i-1);
        else
           outputVektor(i) = inputVektor(i); 
        end
    end
end

