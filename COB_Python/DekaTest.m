classdef DekaTest < handle
    
    % This function communicates with deka_test.py and is for testing deka
    % hand communication
    %
    % Version: 20210219 Author: Tyler Davis
    
    properties
        UDP; UDPEvnt; Status; Event; ClientAddr; ServerAddr;
    end
    
    methods
        
        function obj = DekaTest(varargin)
            obj.ClientAddr = '192.168.42.129';
            obj.ServerAddr = '192.168.42.1';
            obj.Status.ElapsedTime = nan;
            obj.Status.ElapsedTimePy = nan;
            obj.Status.CalcTimePy = nan;
            obj.Status.CurrTime = clock;
            obj.Status.LastTime = clock;
            init(obj);
        end
        
        function init(obj,varargin)
            delete(instrfindall);
            obj.UDP = udp(obj.ServerAddr,20004,'localhost',obj.ClientAddr,'localport',20003,'DatagramReceivedFcn',@obj.read); 
            fopen(obj.UDP); pause(1);
            flushinput(obj.UDP); pause(0.1);
            flushoutput(obj.UDP); pause(0.1);
        end
        
        function close(obj,varargin)
            if isobject(obj.UDP)
                fwrite(obj.UDP,typecast(single([1;zeros(16,1)]),'uint8'),'uint8'); pause(0.1); 
                fclose(obj.UDP); delete(obj.UDP);
            end
        end
        
        function read(obj,varargin)
            try
                val = typecast(uint8(fread(obj.UDP)),'single');
                obj.Status.ElapsedTimePy = val(1);
                obj.Status.ElapsedTime = etime(obj.Status.CurrTime,obj.Status.LastTime);
                obj.Status.LastTime = obj.Status.CurrTime;
            catch
                disp('fread error...')
            end
        end
        
        function write(obj,varargin)
%             msg = unicode2native(varargin{1},'UTF-8');
%             fwrite(obj.UDP,msg);
            fwrite(obj.UDP,typecast(single([0;varargin{1}(:)]),'uint8'),'uint8');
        end
        
    end   
    
end %class


