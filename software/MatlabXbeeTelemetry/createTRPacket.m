function [TRpacket] = createTRPacket(address, data)

    TRpacket=[126 0 (14+size(data,2)) 16 0 address 255 254 0 0 data];
    checksum=255-mod(sum(TRpacket(4:end)),256);
    TRpacket=[TRpacket checksum];
    
end