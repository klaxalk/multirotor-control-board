function [ outputVektor ] = integrate(inputVektor, dt)

    if (nargin == 1)

        outputVektor(1, 1) = 0;

        for i = 2:1:length(inputVektor)
            outputVektor(i, 1) = inputVektor(i)+outputVektor(i-1);
        end
        
    elseif (nargin == 2)
            
        outputVektor(1, 1) = 0;
        
        if (length(dt) > 1)

            for i = 2:1:length(inputVektor)
                outputVektor(i, 1) = inputVektor(i)*dt(i) + outputVektor(i-1);
            end
            
        else
            
            for i = 2:1:length(inputVektor)
                outputVektor(i, 1) = inputVektor(i)*dt + outputVektor(i-1);
            end
            
        end
            
    end
    
end