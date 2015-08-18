function [ output, integral, lastError ] = pidController( error, lastError, integral, P, I, D, antiwindup)

    integral = integral + error;

    if (integral > antiwindup)
        integral = antiwindup;
    elseif (integral < -antiwindup)
        integral = -antiwindup;
    end
    
    output = error*P + integral*I + (error - lastError)*D;

    lastError = error;
    
end
