if exist('COM')
    fclose(COM);
    delete(COM);
    clear COM;
end

clear all

global COM;

COM = serial('COM3');
set(COM,'BaudRate', 115200);
COM.DataBits = 8;
COM.Parity = 'none';
COM.StopBits = 1;
fopen(COM);

xbeeAdresses{1} = [0 hex2dec('13') hex2dec('A2') 0 hex2dec('40') hex2dec('B5') hex2dec('99') hex2dec('BF')];
xbeeAdresses{2} = [0 hex2dec('13') hex2dec('A2') 0 hex2dec('40') hex2dec('B5') hex2dec('99') hex2dec('B4')];
xbeeAdresses{3} = [0 hex2dec('13') hex2dec('A2') 0 hex2dec('40') hex2dec('B5') hex2dec('99') hex2dec('BD')];
xbeeAdresses{4} = [0 hex2dec('13') hex2dec('A2') 0 hex2dec('40') hex2dec('B5') hex2dec('99') hex2dec('BB')];

cibulka = 1;  

figure(1);  
cla
drawnow

while COM.bytesavailable > 0
    
    nic = fread(COM, 1);
    
end

timeStamps(1, 1:4) = 0;

startTime = posixtime(datetime);

while true
    
    actualTime = posixtime(datetime) - startTime;
    
    for i=4:4
        if ((posixtime(datetime) - (4+i*0.5)) > timeStamps(i))

            packet = createTRPacket(xbeeAdresses{i}, ['M' typecast(uint32(actualTime), 'uint8')]);
            
            fprintf('UAV %d is out\n', i);
            
            write(packet);
            timeStamps(i) = posixtime(datetime);
       end
    end
        
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
            
            if (data(1) == 88)
               fprintf('broadcast');
               continue; 
            end
            
            numOfBlobs = data(2);

            idx = 2;
            
            posx = typecast(uint8(data(idx:idx+3)), 'single');
            idx = idx + 4;
            posy = typecast(uint8(data(idx:idx+3)), 'single');
            idx = idx + 4;
            posz = typecast(uint8(data(idx:idx+3)), 'single');
            idx = idx + 4;
            
            setx = typecast(uint8(data(idx:idx+3)), 'single');
            idx = idx + 4;
            sety = typecast(uint8(data(idx:idx+3)), 'single');
            idx = idx + 4;
            setz = typecast(uint8(data(idx:idx+3)), 'single');
            idx = idx + 4;
            
            if (from == hex2dec('B4'))
%                 subplot(1, 3, 1);
                packet = createTRPacket(xbeeAdresses{2}, ['M' typecast(uint32(actualTime), 'uint8')]);
                timeStamps(2) = posixtime(datetime);
                fprintf('received telemetry\n');
            elseif (from == hex2dec('BD'))
%                 subplot(1, 3, 3);
                packet = createTRPacket(xbeeAdresses{3}, ['M' typecast(uint32(actualTime), 'uint8')]);
                timeStamps(3) = posixtime(datetime);
            elseif (from == hex2dec('BF'))
%                 subplot(1, 3, 2);
                packet = createTRPacket(xbeeAdresses{1}, ['M' typecast(uint32(actualTime), 'uint8')]);
                timeStamps(1) = posixtime(datetime);
            elseif (from == hex2dec('BB'))
%                 subplot(1, 3, 2);
                packet = createTRPacket(xbeeAdresses{4}, ['M' typecast(uint32(actualTime), 'uint8')]);
                timeStamps(4) = posixtime(datetime);
            end
            

            figure(1);  
            scatter3(-posy, posx, posz, 'filled', 'LineWidth', 0.1);
            osy = 3;
            axis([-posy-osy, -posy+osy, posx-osy, posx+osy, 0, 2]);
            grid on;
            view([-30 26]);
            drawnow;
            
            [setx, sety, setz]
            
            write(packet);
                       
        end  
        
   end     
   
end
