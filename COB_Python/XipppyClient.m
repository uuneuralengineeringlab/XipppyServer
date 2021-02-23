classdef XipppyClient < handle
    
    % This function communicates with XipppyServer.py running in python
    %
    % XC = XipppyClient; %initialize
    % XC.write(1); %send values to XipppyServer.py
    %
    % Version: 20200826
    % Author: Tyler Davis
    
    properties
        UDP; UDPEvnt; Status; Event; ClientAddr; ServerAddr;
    end
    
    methods
        function obj = XipppyClient(varargin)
            if nargin
                switch varargin{1}
                    case 'windows'
                        obj.ClientAddr = 'localhost';
                        obj.ServerAddr = 'localhost';
                    case 'lan'
                        obj.ClientAddr = '192.168.42.129';
                        obj.ServerAddr = '192.168.42.1';
                    case 'tablet'
                        obj.ClientAddr = '192.168.43.132';
                        obj.ServerAddr = '192.168.43.1';
                    otherwise %nomad wifi
                        obj.ClientAddr = '192.168.43.132';
                        obj.ServerAddr = '192.168.43.1';
                end
            else
                obj.ClientAddr = '192.168.43.132';
                obj.ServerAddr = '192.168.43.1'; %nomad (or windows if testing locally)
            end
            obj.Status.Features = zeros(1,48);
            obj.Status.Kinematics = zeros(1,6);
            obj.Status.Xhat = zeros(1,6);
            obj.Status.Sensors = zeros(1,19);
            obj.Status.ElapsedTime = nan;
            obj.Status.ElapsedTimePy = nan;
            obj.Status.CalcTimePy = nan;
            obj.Status.CurrTime = clock;
            obj.Status.LastTime = clock;
            init(obj);
        end
        function init(obj,varargin)
            delete(instrfindall);
            
            obj.UDP = udp(obj.ServerAddr,20001,'localhost',obj.ClientAddr,'localport',20002,'DatagramReceivedFcn',@obj.read); 
            obj.UDP.InputBufferSize = 65535; obj.UDP.InputDatagramPacketSize = 13107; obj.UDP.OutputBufferSize = 65535; obj.UDP.OutputDatagramPacketSize = 13107;
            fopen(obj.UDP); pause(1);
            flushinput(obj.UDP); pause(0.1);
            flushoutput(obj.UDP); pause(0.1);
            
            obj.UDPEvnt = udp(obj.ServerAddr,20005,'localhost',obj.ClientAddr,'localport',20006,'DatagramReceivedFcn',@obj.readEvnt); 
            obj.UDPEvnt.InputBufferSize = 65535; obj.UDPEvnt.InputDatagramPacketSize = 13107; obj.UDPEvnt.OutputBufferSize = 65535; obj.UDPEvnt.OutputDatagramPacketSize = 13107;
            fopen(obj.UDPEvnt); pause(1);
            flushinput(obj.UDPEvnt); pause(0.1);
            flushoutput(obj.UDPEvnt); pause(0.1);
        end
        function close(obj,varargin)
            if isobject(obj.UDP)
%                 obj.write('close'); pause(0.1); % MP commented: don't
%                 need this to close XipppyServer anymore
                fclose(obj.UDP); delete(obj.UDP);
            end
        end
        function read(obj,varargin)
            try
                val = typecast(uint8(fread(obj.UDP)),'single');
                obj.Status.ElapsedTimePy = val(1);
                obj.Status.CalcTimePy = val(2);
                obj.Status.Features = val(3:50);
                obj.Status.Kinematics = val(51:56);
                obj.Status.Xhat = val(57:62);
                obj.Status.Sensors = val(63:81);
                obj.Status.CurrTime = clock;
                obj.Status.ElapsedTime = etime(obj.Status.CurrTime,obj.Status.LastTime);
                obj.Status.LastTime = obj.Status.CurrTime;
%                 disp(obj.Status.ElapsedTime);
%                 clc; disp(obj.Status.Features');
            catch
                disp('fread error...')
            end
        end
        function readEvnt(obj,varargin)
            try
                val = char(fread(obj.UDPEvnt))';
                obj.Event = val;
            catch
                disp('fread error...')
            end
        end
        function write(obj,varargin)
%             fwrite(obj.UDP,varargin{1});
%             fwrite(obj.UDP,typecast(single(varargin{1}(:)),'uint8'),'uint8'); 
            msg = unicode2native(varargin{1},'UTF-8');
            fwrite(obj.UDP,msg);
        end
    end    
end %class


%%
% u = udp('127.0.0.1',20001,'localhost','127.0.0.1','localport',20002); 
% fopen(u);
% 
% %%
% fwrite(u,1); %fprintf writes a termination character (fwrite does not)
% 
% %%
% fclose(u);
% delete(u);