%% Initialize
clear all;
myScope = oscilloscope();
availableResources = resources(myScope);
% driverlist = drivers(myScope);
% myScope.Resource = availableResources{1};
myScope.Resource = availableResources{2};
connect(myScope);
myScope.Timeout = 1;

% myScope.WaveformLength = 2000;
% myScope.SingleSweepMode = 'off';

enableChannel(myScope, 'CH1');

%% Clear instrument if oscilliscope throws errors
delete(myScope);




%% Record waveforms
xippmex_1_12();

fH = figure;
aH = axes('parent',fH);

%%
cla(aH);
title(aH,'');
drawnow;
WF = nan(32,2500); %chan by sample (2500 is default for oscilloscope)
FrontEndNumber = 3;
pause(5);
for k=[1:2:32,2:2:32]
    disp(k)
    beep;
    StimCommand = ['Elect=',num2str(k+(32*(FrontEndNumber-1))),',;TL=10.0,;Freq=100.0,;Dur=0.2,;Amp=',num2str(100-k),',;TD=0.0,;FS=0.0,;PL=1,;'];
    xippmex_1_12('stim',StimCommand);
    waveformArray = readWaveform(myScope);
    WF(k,:) = waveformArray;
    SC{k} = StimCommand;
    plot(aH,waveformArray)
    title(aH,k)
    axis(aH,[1,2500,-5,5])
    drawnow;
end
save(["StimVerification_"+num2str(FrontEndNumber)],'WF','SC');


%% Python environment doesn't seem correct. Couldn't get it working.
pyenv
P = py.sys.path;
insert(P,int32(0),'C:\\ProgramData\\Anaconda3\\envs\\py36\\Lib\\site-packages');
xp = py.importlib.import_module('xipppy');





