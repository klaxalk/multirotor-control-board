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
            
            numOfBlobs = data(1);
            
            idx = 2;
            
            posx = typecast(uint8(data(idx:idx+3)), 'single');
            idx = idx + 4;
            posy = typecast(uint8(data(idx:idx+3)), 'single');
            idx = idx + 4;
            
            figure(1);
            hold off
            scatter(-posy, posx, 'filled');
            hold on
            
            blobs = zeros(3, numOfBlobs);
            for i=1:numOfBlobs
               for j=1:3
                  blobs(j, i) = typecast(uint8(data(idx:idx+3)), 'single');
                  idx = idx + 4;
               end
               
               circle(-blobs(2, i)-posy, blobs(1, i)+posx, 0.1); 
            end
            
            blobs
            osy = 5;
            axis([-posy-osy, -posy+osy, posx-osy, posx+osy]);
            drawnow;
            
            write(packet);
            
            
        end   
   end       
end

