if exist('COM')
    fclose(COM);
    delete(COM);
    clear COM;
end

clear all

global COM;

COM = serial('COM19');
set(COM,'BaudRate', 115200);
fopen(COM);

uav3 = [0 hex2dec('13') hex2dec('A2') 0 hex2dec('40') hex2dec('B5') hex2dec('99') hex2dec('BD')];
uav2 = [0 hex2dec('13') hex2dec('A2') 0 hex2dec('40') hex2dec('B5') hex2dec('99') hex2dec('B4')];
uav1 = [0 hex2dec('13') hex2dec('A2') 0 hex2dec('40') hex2dec('B5') hex2dec('99') hex2dec('BF')];

addr = uav2;

packet = createTRPacket(addr, ['M']);
write(packet);

figure(1);

tic
time = [];

while true
    
   if COM.bytesavailable > 0
       
        packetIN = fread(COM, 1); 
        
        if packetIN==hex2dec('7E')
            
            hlp = fread(COM, 2)';
            packetIN = [packetIN hlp];
            hlp = fread(COM, packetIN(3) + 1)'; 
            packetIN = [packetIN hlp];
            dec2hex(packetIN);
            data = packetIN(16:end-1);
            
            numOfBlobs = data(2);
            
            idx = 3;
            
            posx = typecast(uint8(data(idx:idx+3)), 'single');
            idx = idx + 4;
            posy = typecast(uint8(data(idx:idx+3)), 'single');
            idx = idx + 4;
            
            setx = typecast(uint8(data(idx:idx+3)), 'single');
            idx = idx + 4;
            sety = typecast(uint8(data(idx:idx+3)), 'single');
            idx = idx + 4;
            
            figure(1);
            hold off
            plot(0, 0);
            hold on
            drawUAV(-posy, posx, numOfBlobs);
            scatter(-sety, setx, 75, 'k', 'filled'); 

            blobs = zeros(2, numOfBlobs);
            for i=1:numOfBlobs
               for j=1:2
                  blobs(j, i) = typecast(uint8(data(idx:idx+3)), 'single');
                  idx = idx + 4;
               end
               
               if (i == 1)
                blobColor = 'r';
               elseif (i == 2)
                blobColor = 'b';
               elseif (i == 3)
                blobColor = 'k';
               end
               scatter(-blobs(2, i)-posy, blobs(1, i)+posx, 150, blobColor, 'filled'); 
            end
            
            blobs
            osy = 5;
%             grid minor;
            axis([-posy-osy, -posy+osy, posx-osy, posx+osy]);
            axis equal;
            set(gca,'xtick',[(round(-posy)-2*osy):1:(round(-posy)+2*osy)]);
            set(gca,'ytick',[(round(posx)-2*osy):1:(round(posx)+2*osy)]);
            grid on;
            drawnow;
            
            write(packet);
                       
        end   
   end       
end

