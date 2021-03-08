function writeWTSfile_matlab()
%%%% This file will take a built training sequence (as a .mat file) and write the header and
%%%% sequence to a wts file. The WTS file is read in by the compiled
%%%% trainDecodeCOB.m which is called within COB_EncodeDecode
%%%% .mat files are made with BuildTrainingMatrixPerDOF.m

%%% This code follows the outline of the writeEncode.m and writeKTF.m files
%% load the sequences built with the BuildTrainingMatrixPerDOF.m file
% load C:\Users\Administrator\Documents\COB_DevFolder\DEKA5_4trials_noThumbInt_noCombo.mat
load C:\Users\Administrator\Code\COB_DevFolder\TrainingSequenceFiles\5DOF_10Trials.mat
timeD = size(KalmanMat,2);
dofsD = size(KalmanMat,1);

%% write the new .WTS file for the Nomad to use in training
fnameWTS = inputdlg('Name the Training Sequence','Write Training Sequence File: *.WTS');
% newfid = fopen(['C:\Users\Administrator\Code\General\Ripple\COB\TrainingSequenceFiles\',fnameWTS{:},'.WTS'],'w+');
newfid = fopen(['C:\Users\Administrator\Code\COB_DevFolder\TrainingSequenceFiles\',fnameWTS{:},'.WTS'],'w+');
fwrite(newfid, datevec(datetime),'single')
fwrite(newfid, [dofsD; timeD; dofsD; timeD],'single');
fwrite(newfid, [1; numel(KalmanMat); numel(DEKAMat)],'single');
fwrite(newfid, [KalmanMat(:); DEKAMat(:)],'single');
fclose(newfid);
end