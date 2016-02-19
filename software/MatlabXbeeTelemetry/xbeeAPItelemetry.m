if exist('COM')
    fclose(COM);
    delete(COM);
    clear COM;
end

clear all

global COM;

COM = serial('COM4');
set(COM,'BaudRate', 115200);
fopen(COM);

uav2 = [0 hex2dec('13') hex2dec('A2') 0 hex2dec('40') hex2dec('B5') hex2dec('99') hex2dec('B4')];
uav3 = [0 hex2dec('13') hex2dec('A2') 0 hex2dec('40') hex2dec('B5') hex2dec('99') hex2dec('BD')];
uav4 = [0 hex2dec('13') hex2dec('A2') 0 hex2dec('40') hex2dec('B5') hex2dec('99') hex2dec('BB')];
uav1 = [0 hex2dec('13') hex2dec('A2') 0 hex2dec('40') hex2dec('B5') hex2dec('99') hex2dec('BF')];

packet = createTRPacket(uav1, ['M']);
write(packet);

packet = createTRPacket(uav2, ['M']);
write(packet);

packet = createTRPacket(uav3, ['M']);
write(packet);

packet = createTRPacket(uav4, ['M']);
write(packet);


figure(1);

tic
time = [];
rssi = [];

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
            from = packetIN(12);
            
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
            
            % pro Cibulku             
            pes = uint8(data(idx));
            idx = idx + 1;
            
            kocka = typecast(uint8(data(idx:idx+3)), 'single');
            idx = idx + 4;
            
            setx;
            sety;
            pes
            kocka
            
            figure(1);
            if (from == hex2dec('B4'))
                subplot(1, 3, 1);
                packet = createTRPacket(uav2, ['M']);
            elseif ((from == hex2dec('BD')))
                subplot(1, 3, 3);
                packet = createTRPacket(uav3, ['M']);
            elseif (from == hex2dec('BF')) | (from == hex2dec('BB'))
                subplot(1, 3, 2);
                packet = createTRPacket(uav1, ['M']);
                packet = createTRPacket(uav4, ['M']);
            end
            
            hold off
            plot(0, 0);
            hold on
            drawUAV(-posy, posx, numOfBlobs);

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
            
            scatter(-sety, setx, 75, 'g', 'filled'); 
            
            blobs;
            osy = 5;
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

