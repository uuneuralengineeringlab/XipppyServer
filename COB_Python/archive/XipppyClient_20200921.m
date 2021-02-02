classdef XipppyClient < handle
    
    % This function communicates with XipppyServer.py running in python
    %
    % XC = XipppyClient; %initialize
    % XC.write(1); %send values to XipppyServer.py
    %
    % Version: 20200826
    % Author: Tyler Davis
    
    properties
        UDP; Status;
    end
    
    methods
        function obj = XipppyClient
            obj.Status.Features = [];                   
            obj.Status.ElapsedTime = nan;
            obj.Status.ElapsedTimePy = nan;
            obj.Status.CalcTimePy = nan;
            obj.Status.CurrTime = clock;
            obj.Status.LastTime = clock;
            init(obj);
        end
        function init(obj,varargin)
            delete(instrfindall);
            obj.UDP = udp('192.168.42.1',20001,'localhost','192.168.42.131','localport',20002,'DatagramReceivedFcn',@obj.read); 
            obj.UDP.InputBufferSize = 65535; obj.UDP.InputDatagramPacketSize = 13107; obj.UDP.OutputBufferSize = 65535; obj.UDP.OutputDatagramPacketSize = 13107;
            fopen(obj.UDP); pause(1);
            flushinput(obj.UDP); pause(0.1);
            flushoutput(obj.UDP); pause(0.1);
        end
        function close(obj,varargin)
            if isobject(obj.UDP)
                obj.write('close'); pause(0.1);
                fclose(obj.UDP); delete(obj.UDP);
            end
        end
        function read(obj,varargin)
            try
                val = typecast(uint8(fread(obj.UDP)),'single');
                obj.Status.ElapsedTimePy = val(1);
                obj.Status.CalcTimePy = val(2);
                obj.Status.Features = val(3:end);
                obj.Status.CurrTime = clock;
                obj.Status.ElapsedTime = etime(obj.Status.CurrTime,obj.Status.LastTime);
                obj.Status.LastTime = obj.Status.CurrTime;
%                 disp(obj.Status.ElapsedTime);
%                 clc; disp(obj.Status.Features');
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