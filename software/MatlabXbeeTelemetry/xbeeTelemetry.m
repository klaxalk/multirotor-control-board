if exist('s')
    fclose(s);
    delete(s);
    clear s;
end

clear all

s = serial('COM23');
s.BaudRate = 115200;
s.BytesAvailableFcnMode = 'terminator';
s.Terminator = 'CR/LF';
s.DataBits = 8;
s.Parity = 'none';
s.StopBits = 1;

fopen(s);

pos = 2;

% receeding horizon length
horizon_length = 100;

in(1:horizon_length, 1:4) = 0;

figure(11);

while true
    
    % request data from a helicopter        
    fprintf(s, '%c', 'b');
    
    % wait for data       
    while (s.BytesAvailable == 14)
    end

    in(pos, 1) = fread(s, 1, 'single');
    in(pos, 2) = fread(s, 1, 'single');
    in(pos, 3) = fread(s, 1, 'single');
    in(pos, 4) = fread(s, 1, 'single');
    in(pos, 5) = fread(s, 1, 'single');
    in(pos, 6) = fread(s, 1, 'single');
    in(pos, 7) = fread(s, 1, 'single');
    in(pos, 8) = fread(s, 1, 'single');
    in(pos, 9) = fread(s, 1, 'int16');
    in(pos, 10) = fread(s, 1, 'int16');
   
    std(in(:, 1))
    
    % read line ending     
    fread(s, 1, 'int16');
    
    % plot the position     
    subplot(2, 1, 1);
    scatter(-in(pos, 2), in(pos, 1), 'filled');
    axis([-1 1 -1 1]);
    
    % plot the action
    subplot(2, 1, 2);
    hold off
    plot(in([pos:end 1:pos-1], 9), 'g');
%     plot(in([pos:end 1:pos-1], 1), 'b');
    hold on
    plot(in([pos:end 1:pos-1], 10), 'b');
%     plot(in([pos:end 1:pos-1], 3), 'r');
    axis([0 horizon_length -150 150]);
    
% 
%     s3 = subplot(2, 1, 2);
%     cla(s3)
%     hold off
%     text(.1,.5,['\fontsize{16}Øízení 1. úrovnì: ']);
%     if (controllerEnabled == 1)
%         text(.5,.5,['\fontsize{16}\color{green}Zapnuto ']);
%     else
%         text(.5,.5,['\fontsize{16}\color{red}Vypnuto ']);
%     end
%     text(.1,0,['\fontsize{16}Øízení 2. úrovnì: ']);
%     if (positionControllerEnabled == 1)
%         text(.5,0,['\fontsize{16}\color{green}Zapnuto ']);
%     else
%         text(.5,0,['\fontsize{16}\color{red}Vypnuto ']);
%     end
% 
%     if (landing == 1)
%         text(.1,-0.5,['\fontsize{16}\color{green}Pøistání ']);
%     end
%     
%     if (trajectory == 1)
%         text(.1,-0.5,['\fontsize{16}\color{green}Trajektorie ']);
%     end
%     if (gumstix == 1)
%         text(.5,-0.5,['\fontsize{16}\color{green}Gumstix ']);
%     end
%     axis([0 0.8 -1 1]);

    drawnow;

    pos = pos + 1;
    if (pos == (horizon_length+1))
       pos = 1; 
    end
    
end