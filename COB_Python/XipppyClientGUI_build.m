%% Run this to create FeedbackDecode.exe  
% -a means include folder and files in compilation
mcc -m XipppyClientGUI.m -v -N -p instrument -p signal -p stats -p curvefit -p images -p mbc -p nnet
