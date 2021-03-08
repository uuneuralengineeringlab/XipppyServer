function BuildTrainingMatrix() %#codegen
%%% This function is adapted from
%%% TrainTest_EMG_Neural_Nomad_DOFSel_allVel.m as of 8/14/18 
%%% This function is only for EMG, training and testing on the nomad, no
%%% auto channel selection or differential pairs, but just map the 32
%%% electrode features to the output of the SS KF.

%% Structure to pass to the DEKA for mvnts
    pos = struct('index_finger', 0, 'wrist_pron', 0, 'wrist_flex', 0, ...
        'mrp', 0, 'thumb_yaw', 0, 'thumb_pinch', 0);
% timing init
    CycleTime = 0.033; %(33ms)
%%
numTrials = 4;
numDOFs = 6;
%%
% MvntDOFs = cell(numMvnts,1); %%% logical, which DOFs are in each mvnt
% DelayDOFs = cell(numMvnts,1); %%% logical, which DOFs are in each mvnt
% SkipNegMvnt = false(numMvnts,1);  %%% if true the negative mvnt is not included
% HoldTimes = zeros(numMvnts,1);
% RiseTime = zeros(numMvnts,1);
% Amp = cell(numMvnts,1);
% BtwMvntDelay = zeros(numMvnts,1); %%% delay between each Mvnt
% restPos = zeros(numDOFs,1);

% RiseTime = [0.7;0.7;0.7;0.7;0.7;0.7;0.7];
RiseTime = [0.7];
restPos = [0; 0; 0; 0.6; 0; 0; 0];
PosVel = [0; 0; 0; 0; 1; 1; 0;]; % Pos = 0, Vel =1; %%% According to the DEKA PI interface (set pos modes to vel modes per DOF after this).
% % sigmoid movement
% Pos_y = zeros(numDOFs+1,length(xRise));
% Neg_y = zeros(numDOFs+1,length(xRise));
% a_sig = 1;
% c_sig = 5;
%%
% deka_move_COB(restPos);
% xl_pause(2);

%% For each mvnt make the matrix then put together after
% For each Mvnt input the desired DOFs, delays and holds, if an inverse is included etc...
% this will be code to parse out in inputs to the desired variables =
% '1,4,D1,6,H1,6' = D1 delays 4 from 1/2, all hold at amp then 6 undoes
% earlier 6 using a negative amp.
answer = {'','',''};
jj = 0;
while (~isempty(answer))
    jj = jj+1; %new movement
    prompt = {'Enter sequence of DOFs (i.e. 1,4,D1,6,H1,6):','Enter any delays (less than rise time) relative to start of previous DOFs (i.e. D=0.7,D1 = 1):',...
        'Enter hold durations (i.e. H=1,H1=2):', 'Enter Amplitude for each DOF in sequence (i.e. 1,1,0.5,-0.5):',...
        };
    title = ['Mvnt #',int2str(jj)];
    dims = [1 35];
    definput = {'1,4,H','','H=1','1,1'};
    answer = inputdlg(prompt,title,dims,definput);
    if ~isempty(answer)
        %% Parse inputs
        inputAmps = answer(4);
        inputHolds = answer(3);
        inputDelays = answer(2);
        inputMvnts = answer(1);
        % inputMvnts = {'1,4,D1,6,H1,6'};
        
        pMvnts = split(inputMvnts{:},',');
        MvntDOFs = cellfun(@str2num,pMvnts(cell2mat(cellfun(@(x) ~isempty(str2num(x(1))),pMvnts,'UniformOutput',false))));
        % MvntDOFs = [1,4,6];
        seqDelays = cell2mat(cellfun(@(x) x(1)=='D',pMvnts,'UniformOutput',false));
        seqHolds = cell2mat(cellfun(@(x) x(1)=='H',pMvnts,'UniformOutput',false));
        seqDOFs = ~seqDelays & ~seqHolds;
        if ~isempty(inputDelays{:})
            pDelays = split(inputDelays{:},[",","="]);
            DelayDOFs = cellfun(@str2num, (pDelays(circshift(cell2mat(cellfun(@(x) x(1)=='D', pDelays,'UniformOutput',false)),1))));
        else
            DelayDOFs = zeros(sum(seqDelays),1); %% No delay default
        end
        % DelayDOFs = [0.7]; % in seconds - array is length of unique Delays in the MvntDOFs
        
        if ~isempty(inputHolds{:})
            pHolds = split(inputHolds{:},[",","="]);
            HoldDOFs = cellfun(@str2num, (pHolds(circshift(cell2mat(cellfun(@(x) x(1)=='H', pHolds,'UniformOutput',false)),1))));
        else
            HoldDOFs = ones(sum(seqHolds),1);   %% 1 sec hold default
        end
        % HoldDOFs = [2]; % in seconds - array is the length of the unique Holds in the MnvtDOFs
        if ~isempty(inputAmps{:})
            pAmps = split(inputAmps{:},[","]);
            AmpDOFs = cellfun(@str2num, (pAmps));
        else
            AmpDOFs = ones(size(MvntDOFs));  %% amplitudes of 1 default
        end
        %Amp = [-0.5,1,1];   
        %% Build sigmoid movement
        xRise = linspace(0,10,floor(RiseTime/CycleTime));
        Pos_y = zeros(numDOFs+1,length(xRise));
        Neg_y = zeros(numDOFs+1,length(xRise));
        a_sig = 1;
        c_sig = 5;
        % hold movmement
        for kk = 1:length(HoldDOFs)
            xHold{kk} = linspace(0,10,floor(HoldDOFs(kk)/CycleTime));
        end
        %% Calculate the total time of the movement
        % up to the first delay or hold there is only one rise time, add in
        % the delay first (and any secondary delay if present) until you
        % reach a hold, then add the hold in. Repeat, one rise for all
        % before a hold or delay etc...
        TotalTime = RiseTime*(sum(abs(diff(seqDOFs)))-sum(seqDelays(1:end-1))-sum(seqHolds(1:end-1)));
        for kk = 1:length(seqDOFs)
            if seqHolds(kk)==true()
                TotalTime = TotalTime + HoldDOFs(sum(seqHolds(1:kk)));
            elseif seqDelays(kk) == true()
                TotalTime = TotalTime + DelayDOFs(sum(seqDelays(1:kk)));
            end
        end
        MvntMatDOFs = zeros(numDOFs+1,floor(TotalTime/CycleTime));
        lastTemp = 1;
        for kk = 1:length(seqDOFs)   %%%%% NEXT: figure out how the DOFs and Delays work together to set the lastTemp....
            if seqDOFs(kk) == true()
                MvntMatDOFs(MvntDOFs(sum(seqDOFs(1:kk))),1:length(xRise)) = Pos_y(MvntDOFs(sum(seqDOFs(1:kk))),:);
                lastTemp = lastTemp + floor(RiseTime./CycleTime)-1;
            elseif seqDelays(kk) == true()
            elseif seqHolds(kk) == true()
                tempMat = repmat(MvntMatDOFs(:,lastTemp),1,floor(HoldDOFs(sum(seqHolds(1:kk)))./CycleTime));
                MvntMatDOFs(:,lastTemp+1:lastTemp+size(tempMat,2)) = tempMat;
            end
        end
                
        %% Define movement trajectory with sigmoid
        for kk = 1:length(MvntDOFs)
            PosAmp = AmpDOFs(kk).*(1-restPos(MvntDOFs(kk)));
%             NegAmp = AmpDOFs{jj}(kk).*(-1-restPos(MvntDOFs{jj}(kk)));
            Pos_y(MvntDOFs(kk),:) = PosAmp./(1+exp(-a_sig.*(xRise-c_sig))); %%% Store each trajectory in the MvntDOFs
%             Neg_y(MvntDOFs{jj}(kk),:) = NegAmp./(1+exp(-a_sig.*(xRise-c_sig)));
        end
        
        % sinusoid
        % x = linspace(-pi,0,floor(Rise/CycleTime));
        % y = (cos(x)+1)./2;
        
        %% MvntMat to send to DEKA
        PosWvfrm = [zeros(size(Pos_y,1),floor(StartDur/CycleTime)),Pos_y,...
            repmat(PosAmp.*~PosVel,1,floor(Hold/CycleTime)).*ones(size(Pos_y,1),floor(Hold/CycleTime)),...
            (-1*Pos_y).*repmat(PosVel,1,size(Pos_y,2)),zeros(size(Pos_y,1),floor(EndDelay/CycleTime))];
        PosMvnt = repmat(PosWvfrm,1,NumTrials);
        
        NegWvfrm = [zeros(size(Neg_y,1),floor(StartDur/CycleTime)),Neg_y,...
            repmat(NegAmp.*~PosVel,1,floor(Hold/CycleTime)).*ones(size(Neg_y,1),floor(Hold/CycleTime)),...
            (-1*Neg_y).*repmat(PosVel,1,size(Neg_y,2)),zeros(size(Neg_y,1),floor(EndDelay/CycleTime))];
        NegMvnt = repmat(NegWvfrm,1,NumTrials);
        
        % Grasp
        NegGrasp = NegMvnt;
        PosGrasp = PosMvnt;
        NegGraspB = NegGrasp(4,:);
        PosGraspB = PosGrasp(4,:);
        NegGrasp(5:6,:) = 0; NegGrasp(1,:) = 0; NegGrasp(7,:) = 1; NegGrasp(4,:) = PosGraspB;
        PosGrasp(5:6,:) = 0; PosGrasp(1,:) = 0; PosGrasp(7,:) = 1; PosGrasp(4,:) = NegGraspB;
        NegGrasp1 = NegWvfrm;
        PosGrasp1 = PosWvfrm;
        NegGraspB1 = NegGrasp1(4,:);
        PosGraspB1 = PosGrasp1(4,:);
        NegGrasp1(5:6,:) = 0; NegGrasp1(1,:) = 0; NegGrasp1(7,:) = 1; NegGrasp1(4,:) = PosGraspB1;
        PosGrasp1(5:6,:) = 0; PosGrasp1(1,:) = 0; PosGrasp1(7,:) = 1; PosGrasp1(4,:) = NegGraspB1;
        PosGrasp1 = PosGrasp1+repmat(restPos,1,size(PosGrasp1,2));
        NegGrasp1 = NegGrasp1+repmat(restPos,1,size(NegGrasp1,2));
        
        % Matrix for pretraining
        MvntOnce = [PosWvfrm, zeros(size(Pos_y,1), floor(MvntDelay/CycleTime)), NegWvfrm, zeros(size(Pos_y,1), floor(MvntDelay/CycleTime))];
        Mvnt1D = [MvntOnce(1,:),zeros(1,length(MvntOnce)*5);...
            zeros(1,length(MvntOnce)*1), MvntOnce(2,:),zeros(1,length(MvntOnce)*4);...
            zeros(1,length(MvntOnce)*2), MvntOnce(3,:),zeros(1,length(MvntOnce)*3);...
            zeros(1,length(MvntOnce)*3), MvntOnce(4,:),zeros(1,length(MvntOnce)*2);...
            zeros(1,length(MvntOnce)*4), MvntOnce(5,:),zeros(1,length(MvntOnce)*1);...
            zeros(1,length(MvntOnce)*5), MvntOnce(6,:);...
            ones(1,(length(MvntOnce)*6))];
        Mvnt1D = Mvnt1D + repmat(restPos,1,size(Mvnt1D,2));
        Mvnt1D = [Mvnt1D, PosGrasp1, NegGrasp1];
        Mvnt = [PosMvnt,zeros(size(Pos_y,1),floor(MvntDelay/CycleTime)),NegMvnt, zeros(size(Pos_y,1),floor(MvntDelay/CycleTime))]; % Flex followed by extend
        MvntMat = double([Mvnt(1,:), zeros(1,(length(Mvnt)*(numDOFs-1)));...
            zeros(1,(length(Mvnt)*(numDOFs-5))), Mvnt(2,:), zeros(1,(length(Mvnt)*(numDOFs-2)));...
            zeros(1,(length(Mvnt)*(numDOFs-4))), Mvnt(3,:), zeros(1,(length(Mvnt)*(numDOFs-3)));...
            zeros(1,(length(Mvnt)*(numDOFs-3))), Mvnt(4,:), zeros(1,(length(Mvnt)*(numDOFs-4)));...
            zeros(1,(length(Mvnt)*(numDOFs-2))), Mvnt(5,:), zeros(1,(length(Mvnt)*(numDOFs-5)));...
            zeros(1,(length(Mvnt)*(numDOFs-1))), Mvnt(6,:);...
            ones(1,(length(Mvnt)*(numDOFs)))]);
        MvntMat =  [MvntMat,PosGrasp,NegGrasp];
        MvntMat = MvntMat + repmat(restPos,1,size(MvntMat,2));
        %% TrainMat to send to Kalman (different because we control the wrist in velocity)
        PosTrainWvfrm = [zeros(size(Pos_y,1),floor(StartDur/CycleTime)),Pos_y,...
            repmat(PosAmp,1,floor(Hold/CycleTime)).*ones(size(Pos_y,1),floor(Hold/CycleTime)),...
            (-1*Pos_y+repmat(PosAmp,1,size(Pos_y,2))).*repmat(PosVel,1,size(Pos_y,2)),...
            zeros(size(Pos_y,1),floor(EndDelay/CycleTime))];
        PosTrainMvnt = repmat(PosTrainWvfrm,1,NumTrials);
        
        NegTrainWvfrm = [zeros(size(Neg_y,1),floor(StartDur/CycleTime)),Neg_y,...
            repmat(NegAmp,1,floor(Hold/CycleTime)).*ones(size(Neg_y,1),floor(Hold/CycleTime)),...
            (-1*Neg_y+repmat(NegAmp,1,size(Pos_y,2))).*repmat(PosVel,1,size(Neg_y,2)),...
            zeros(size(Neg_y,1),floor(EndDelay/CycleTime))];
        NegTrainMvnt = repmat(NegTrainWvfrm,1,NumTrials);
        
        TrainMvnt = [PosTrainMvnt,zeros(size(Pos_y,1),floor(MvntDelay/CycleTime)),NegTrainMvnt, zeros(size(Pos_y,1),floor(MvntDelay/CycleTime))]; % Flex followed by extend
        TrainMat = double([TrainMvnt(1,:), zeros(1,(length(Mvnt)*(numDOFs-1)));...
            zeros(1,(length(Mvnt)*(numDOFs-5))), TrainMvnt(2,:), zeros(1,(length(Mvnt)*(numDOFs-2)));...
            zeros(1,(length(Mvnt)*(numDOFs-4))), TrainMvnt(3,:), zeros(1,(length(Mvnt)*(numDOFs-3)));...
            zeros(1,(length(Mvnt)*(numDOFs-3))), TrainMvnt(4,:), zeros(1,(length(Mvnt)*(numDOFs-4)));...
            zeros(1,(length(Mvnt)*(numDOFs-2))), TrainMvnt(5,:), zeros(1,(length(Mvnt)*(numDOFs-5)));...
            zeros(1,(length(Mvnt)*(numDOFs-1))), TrainMvnt(6,:);...
            ones(1,(length(Mvnt)*(numDOFs)))]);
        TrainMat = [TrainMat,PosGrasp,NegGrasp];
        TrainMat = TrainMat + repmat(restPos,1,size(TrainMat,2));
        %% %short Training MvntMat 3DOFs
        % MvntMat = double([Mvnt, zeros(1,(length(Mvnt)*(numDOFs-3)));...
        %     zeros(1,(length(Mvnt)*(numDOFs-5))), Mvnt, zeros(1,(length(Mvnt)*(numDOFs-4)));...
        %     zeros(1,(length(Mvnt)*(numDOFs-4))), Mvnt, zeros(1,(length(Mvnt)*(numDOFs-5)));...
        %     zeros(1,(length(Mvnt)*(numDOFs-2)));...
        %     zeros(1,(length(Mvnt)*(numDOFs-2)));...
        %     zeros(1,(length(Mvnt)*(numDOFs-2)))]);
        % %short Training MvntMat All DOFs combo
        % MvntMat = double([Mvnt, zeros(1,(length(Mvnt)*(numDOFs-6)));...
        %     Mvnt, zeros(1,(length(Mvnt)*(numDOFs-6)));...
        %     Mvnt, zeros(1,(length(Mvnt)*(numDOFs-6)));...
        %     Mvnt, zeros(1,(length(Mvnt)*(numDOFs-6)));...
        %     Mvnt, zeros(1,(length(Mvnt)*(numDOFs-6)));...
        %     Mvnt, zeros(1,(length(Mvnt)*(numDOFs-6)))]);
        % %shortest Training MvntMat 1DOF
        % MvntMat = double([Mvnt;...
        %     zeros(1,(length(Mvnt)));...
        %     zeros(1,(length(Mvnt)));...
        %     zeros(1,(length(Mvnt)));...
        %     zeros(1,(length(Mvnt)));...
        %     zeros(1,(length(Mvnt)))]);
    end
end
%% Initialize Training Parameters
    NIPTime = zeros(1,size(MvntMat,2));
    durTotal = 300;
    BufferInd = (1:durTotal);
% get Matrix for EMG differential pairs
    [DiffMat,EMGChanPairs,~] = genEMGDiffMatrix([]);  % 32 x 528
% find and remove single ended
    EMGMatrix = zeros(size(DiffMat,2)-length(EMGchans),length(EMGchans));
    ii = 1;
    for i = 1:size(EMGChanPairs,1)
        if isnan(EMGChanPairs(i,2))
        else
            EMGMatrix(ii,:) = DiffMat(:,i);
            ii = ii+1;
        end
    end
    EMGBuffer = zeros(size(EMGMatrix,1),durTotal); %%% only for diff
    TrainMatrix = zeros(size(EMGBuffer,1)+length(Nchans),size(MvntMat,2));

