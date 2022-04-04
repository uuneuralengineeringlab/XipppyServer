function TrainingKDF2WTS(kdfPath, WTSname)
% Converts a training file (.kdf) from FeedbackDecode to a WTS file for use
% with Nomad. 
% 
% example: 
% kdfPath = "D:\FeedbackDecode\20210112-162636\TrainingData_20210112-162636_164414.kdf";
% WTSname = "6DOF4Trl";
% TrainingKDF2WTS(kdfPath, WTSname);


[Kinematics,~,~,~,~] = readKDF(kdfPath);

% order of kin in FeedbackDecode: thumb, ind, mid, ring, lit, thumbInt, indexInt, ringInt,
% littleInt, wristFE, wristYaw, wristRoll

% order of kin in XipppyServer: thumb, ind, mrp, thumbint, wristfe, wristrot

XS_kin = Kinematics([1, 2, 3, 6, 10, 12],:);
bias = ones(1,size(XS_kin,2));
XS_kin = [XS_kin; bias];
date = datevec(datetime); 
szVec = repmat(size(XS_kin),1,2);
len1D = repmat(szVec(1)*szVec(2),1,2);

toFile = [date, ...
    szVec, ...
    1, ...
    len1D, ...
    reshape(XS_kin, 1, []), ...
    reshape(XS_kin, 1, [])];

path = fileparts(mfilename('fullpath'));
fullpath = fullfile(path, [char(WTSname) '.wts']);
fid = fopen(fullpath, 'w');

fwrite(fid, toFile, 'single');

fclose(fid);

fprintf('WTS successfully written to %s\nTraining should take approximately %0.1f minutes. \n', fullpath, size(XS_kin,2)/30/60)