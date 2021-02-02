%% Connect to nip
UDPEvnt = udp('192.168.42.1',2046,'localhost','192.168.42.129','localport',2046); %Sending/receiving string commands
UDPEvnt.InputBufferSize = 65535; UDPEvnt.InputDatagramPacketSize = 13107; UDPEvnt.OutputBufferSize = 65535; UDPEvnt.OutputDatagramPacketSize = 13107;

fopen(UDPEvnt);


%% displaying nip timestamp
flushinput(UDPEvnt);
flushoutput(UDPEvnt);
d = zeros(30000,1);
for k=1:30000
data = fread(UDPEvnt);
% disp(typecast(uint8(data(5:8)),'uint32'));
if length(data)==76
%     disp(typecast(uint8(data(13:end)),'int16'))
elseif length(data)==164
%     disp(typecast(uint8(data(end-20:end-19)),'int16'))
    d(k) = typecast(uint8((data([13,14,89,90]))),'int32');
%     d(k) = UDPEvnt.BytesAvailable;
%     data([13,14,89,90]) %channel 1 data??
end
% disp(length(data))
end
UDPEvnt.BytesAvailable

%%
fclose(UDPEvnt);