function [  ] = printMatrix(fid, text, format, inMatrix)

    fprintf(fid, text);
    fprintf(fid, ' = {\n');

    for i=1:size(inMatrix, 1)
        for j=1:size(inMatrix, 2)
            fprintf(fid, format, inMatrix(i, j)); 

            if (j == size(inMatrix, 2))
                fprintf(fid, ', \n');
            else
                fprintf(fid, ', ');
            end
        end
    end

    fprintf(fid, '};\n\n');

end

