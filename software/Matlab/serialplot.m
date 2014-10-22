xlabel('Steps');
ylabel('Data');
title('Serial Link Data Logger');

s = serial('COM8');
set(s, 'FlowControl', 'hardware');
set(s, 'BaudRate', 9600);
set(s, 'Parity', 'none');
set(s, 'DataBits', 8);
set(s, 'StopBit', 1);
set(s, 'Timeout', 2);
fopen(s);

data=0;

drawnow;
plotGraph = plot(data,'-mo',...
                'LineWidth',1,...
                'MarkerEdgeColor','k',...
                'MarkerFaceColor',[.49 1 .63],...
                'MarkerSize',2);            
grid on;

while ishandle(plotGraph)
    if (s.BytesAvailable)
       out = fread(s,4,'float')
       data=[data out'];
       drawnow;
       plotGraph = plot(data, '.-');
    end    
end

fclose(s);