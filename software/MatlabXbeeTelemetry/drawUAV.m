function [ output_args ] = drawUAV(x, y, numOfBlobs)

    viewDist = 4;

    patch([x-viewDist, x+viewDist, x+0.5, x-0.5], [y+viewDist, y+viewDist, y+0.5 y+0.5], [0.97 0.97 0.97], 'FaceAlpha',.4, 'EdgeColor', 'g');
    patch([x-viewDist, x-viewDist, x-0.5, x-0.5], [y+viewDist, y-viewDist, y-0.5 y+0.5], [0.97 0.97 0.97], 'FaceAlpha',.4, 'EdgeColor', 'g');
    patch([x+viewDist, x+viewDist, x+0.5, x+0.5], [y+viewDist, y-viewDist, y-0.5 y+0.5], [0.97 0.97 0.97], 'FaceAlpha',.4, 'EdgeColor', 'g');
    circle(x-0.16, y-0.16, 0.15);
    circle(x-0.16, y+0.16, 0.15);
    circle(x+0.16, y-0.16, 0.15);
    circle(x+0.16, y+0.16, 0.15);
    plot([x-0.17, x+0.17], [y-0.17, y+0.17], 'k');  
    plot([x-0.17, x+0.17], [y+0.17, y-0.17], 'k');
%     plot([x-0.3, x-3], [y+0.3, y+3], 'b--');
%     plot([x+0.3, x+3], [y+0.3, y+3], 'b--');
    text(double(x-0.1), double(y-0.5), int2str(numOfBlobs), 'FontSize', 20);
                  
end

