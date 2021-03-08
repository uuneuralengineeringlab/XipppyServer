function [DEKAMat, KalmanMat] = BuildTrainingMatrixPerDOF()
%% For each DOF in the movement, lay out the sequence then put each matrix together
% For each Mvnt input the desired DOFs, delays and holds, if an inverse is included etc...
% this will be code to parse out in inputs to the desired variables 
% To start run: [DEKAMat,KalmanMat] = BuildTrainingMatrixPerDOF()
showMvnt = false;
if showMvnt
    xl_open_tcp()
end
plotMvnt = false;
fig_mvnt = figure();

% timing init
CycleTime = 0.033; %(33ms)[

% restPos = [-0.8; 0; 0; 0.6; 0; 0; 1]; %% bias term rests at 1 %%% updated to be [ThbFE,IndFE,mrpFE,ThbInt,WrFE,WrPS,bias] 
% restPos = [-0.6;0;0;0.8;0;0;1]; %%%% If this is changed the A's in the training will be off: Training MATS depend on it (not just +1/-1)....
% restPos = [-0.6;0;0;0.6;0;0;1]; %%% 0.6 is about right for the key grip on thumb
restPos = [0;0;0;0;0;0;1]; %%% A proper rest position is accounted for, no need to change
%%% Notes: thumb adduction never needs to go beyond the 0.25 point...it
%%% cant press against the hand at that point anyway and if it's further
%%% extended it will only rotate awkwardly. A=-0.5,A1=-0.4,A2=-0.3,A3=-0.2,A4=-0.1,A5=0
numDOFs = 6;
KalmanMat = restPos;
DEKAMat = KalmanMat;
PosVel = false([1,7]);
PosVel(5:6) = true();% Pos = 0, Vel =1; %%% According to the DEKA PI interface (set pos modes to vel modes per DOF after this).
velFactor = 15;
%
% PosVel = [0; 0; 0; 0; 1; 1; 0;];
answer = repmat({''},1,10);
answerQ = 'Keep';
jj = 0;
while (~isempty(answer))
    if strcmp(answerQ,'Keep')
    jj = jj+1; %new movement
    end
    prompt = {'Name (optional):','Thumb Add/Abd (i.e. R,A,H):','Index:',...
        'Middle Ring Pinky:','Thumb F/E:','Wrist F/E (include final R):','Wrist Rotate (include final R):',...
        'Define Rise Times i.e. R=0.7, R1=1):',...
        'Define Delays (i.e. D=0.7,D1 = 1):',...
        'Define Holds (i.e. H=1,H1=2):',...
        'Define Amplitudes (i.e. A=1, A1=0.5, A2=-0.5):',...
        'Define Inter-Trial Delay:',...
        'Define Number of Trials:',...
        'Define Post Movement Delay:'...
        };
    GUItitle = ['Mvnt #',int2str(jj)];
    dims = [1 35];
    definput = {'Add Name','R,A,H','','','','','','R=0.7,R1=1.5','D=0','H=2','A=1,A1=-1','1','4','1.3'};
    answer = inputdlg(prompt,GUItitle,dims,definput);
    if ~isempty(answer)
        %% Parse waverform inputs
        inputName = answer(1);
        
        pAmps = split(answer{11},[",","="]);
        NumCell = cellfun(@str2num,pAmps,'UniformOutput', false);
        inputAmps = pAmps(cellfun(@isempty,NumCell)); clear NumCell;
        numAmps = cellfun(@str2num, (pAmps(circshift(cell2mat(cellfun(@(x) x(1)=='A', pAmps,'UniformOutput',false)),1))));
        
        pHolds = split(answer{10},[",","="]);
        NumCell = cellfun(@str2num,pHolds,'UniformOutput', false);
        inputHolds = pHolds(cellfun(@isempty,NumCell)); clear NumCell;
        numHolds = cellfun(@str2num, (pHolds(circshift(cell2mat(cellfun(@(x) x(1)=='H', pHolds,'UniformOutput',false)),1))));
        
        pDelays = split(answer{9},[",","="]);
        NumCell = cellfun(@str2num,pDelays,'UniformOutput', false);
        inputDelays = pDelays(cellfun(@isempty,NumCell)); clear NumCell;
        numDelays = cellfun(@str2num, (pDelays(circshift(cell2mat(cellfun(@(x) x(1)=='D', pDelays,'UniformOutput',false)),1))));
        
        pRises = split(answer{8},[",","="]);
        NumCell = cellfun(@str2num,pRises,'UniformOutput', false);
        inputRises = pRises(cellfun(@isempty,NumCell)); clear NumCell;
        numRises = cellfun(@str2num, (pRises(circshift(cell2mat(cellfun(@(x) x(1)=='R', pRises,'UniformOutput',false)),1))));
        
        interTrialDelay = str2num(answer{12});
        numTrials = str2num(answer{13});
        postMvntDelay = str2num(answer{14});
        mvntCell = num2cell(restPos); %%% cell for each mvnt
        
        %% Parse and build each DOF in current mvnt
        for iDOF = 1:6   %%% thumb flex, index, mrp, thumb intrinsic, wrist flex, wrist rotate
            if ~isempty(answer{iDOF+1})
                pMvnts = split(answer{iDOF+1},',');
                seqDelays = cell2mat(cellfun(@(x) x(1)=='D',pMvnts,'UniformOutput',false));
                seqHolds = cell2mat(cellfun(@(x) x(1)=='H',pMvnts,'UniformOutput',false));
                seqRises = cell2mat(cellfun(@(x) x(1)=='R',pMvnts,'UniformOutput',false));
                seqAmps = cell2mat(cellfun(@(x) x(1)=='A',pMvnts,'UniformOutput',false));
                for iseq = 1:length(pMvnts)
                    if seqDelays(iseq)
                        %%% Delay keeps the current value (0 or restPos) for
                        %%% a duration of time
                        DelayTime = numDelays(strcmp(inputDelays,pMvnts(iseq)));
                        x_line = linspace(0,10,floor(DelayTime/CycleTime));
                        mvntCell{iDOF} = [mvntCell{iDOF}, repmat(mvntCell{iDOF}(end),1,length(x_line))];
                    elseif seqHolds(iseq)
                        %%% Hold keeps the previous value for a duration of
                        %%% time; if not followed by a rise or Amp drop it
                        %%% or up it to 0 or restPos at the end
                        HoldTime = numHolds(strcmp(inputHolds,pMvnts(iseq)));
                        x_line = linspace(0,10,floor(HoldTime/CycleTime));
                        mvntCell{iDOF} = [mvntCell{iDOF}, repmat(mvntCell{iDOF}(end),1,length(x_line))];
                    elseif seqRises(iseq)
                        %%% Adds a rise feature (sigmoid) from current location to the next
                        %%% defined amp (if not followed by an amplitude it goes to 0/restPos
                        RiseTime = numRises(strcmp(inputRises,pMvnts(iseq)));
                        x_line = linspace(0,10,floor(RiseTime/CycleTime));
                        if iseq+1 <= length(pMvnts)
                            if seqAmps(iseq+1)
                                AmpDOFs = numAmps(strcmp(inputAmps,pMvnts(iseq+1))); %%% match the amplitude to reach
                            else
                                AmpDOFs = restPos(iDOF);
                            end
                        else
                            AmpDOFs = restPos(iDOF);
                        end
                        Rise_wvfrm = zeros(1,length(x_line));
                        a_sig = 1;
                        c_sig = 5;
                        RiseAmp = AmpDOFs - mvntCell{iDOF}(end); %%% difference between where you are and where you want to go
                        try
                            Rise_wvfrm(:) = RiseAmp./(1+exp(-a_sig.*(x_line-c_sig)))+mvntCell{iDOF}(end); %%% Store each trajectory in the MvntDOFs
                            mvntCell{iDOF} = [mvntCell{iDOF}, Rise_wvfrm,AmpDOFs];
                        catch
                            1
                        end
                    elseif seqAmps(iseq)
                        %%% Amplitude defines where the rise is going to, if
                        %%% not rise does not preceed by default the mvnt
                        %%% jumps to the defined Amp
                        AmpDOFs = numAmps(strcmp(inputAmps,pMvnts(iseq)));
                        mvntCell{iDOF} = [mvntCell{iDOF}, AmpDOFs];
                    end
                end
            else
                %%% no movement from this DOF - all zeros or rest position
                mvntCell{iDOF} = restPos(iDOF);
            end
        end
        
        %% Fill in end, add inter-mvnt delay to make each DOF the same length
        sz_mvntCell = cellfun(@length,mvntCell(:),'UniformOutput',false); %% length of each DOF train
        [mx_mvntCell,maxI] = max([sz_mvntCell{:}]); %% length of maximum DOF train
        tDelay = floor(interTrialDelay/CycleTime);
        mvntCell{maxI} = [mvntCell{maxI},repmat(restPos(maxI),1,tDelay)];
        sz_mvntCell = cellfun(@length,mvntCell(:),'UniformOutput',false); %% Find NEW lengths and max length
        mx_mvntCell = max([sz_mvntCell{:}]); %% length of maximum DOF train
        for iDOF=1:length(mvntCell)
            if iDOF == length(mvntCell)
                mvntCell{iDOF} = ones(1,mx_mvntCell); % set the bias DOF to ones
            else
                if sz_mvntCell{iDOF} < mx_mvntCell
                    mvntCell{iDOF} = [mvntCell{iDOF},repmat(restPos(iDOF),1,(mx_mvntCell-sz_mvntCell{iDOF}))];
                end
            end
        end
        
%% Build the whole sequence of training with numTrials and add the postMvntDelay
        tDelay = floor(postMvntDelay./CycleTime);
        trialsCell = cellfun(@(y,z) [y,repmat(z,1,tDelay)], ...
            cellfun(@(x)repmat(x,1,numTrials),mvntCell,'UniformOutput',false),...
            num2cell(restPos),'UniformOutput',false);
        
%% review submission       
        TempKalmanMat = [cell2mat(trialsCell)];
        TempDEKAMat = TempKalmanMat;
        TempDEKAMat(PosVel',:) = [diff(TempDEKAMat(PosVel',:),1,2)*velFactor, restPos(PosVel')]; %%% velocity factor required to get the hand to move enough to see
        if plotMvnt
            set(0,'currentfigure',fig_mvnt)
            sgtitle(['Mvnt #',int2str(jj),'/',inputName{:}])
            h11 = subplot(7,1,1);
                        plot(TempKalmanMat(1,:))
                        title('Thumb Abb/Add')
            h12 = subplot(7,1,2);
                        plot(TempKalmanMat(2,:))
                        title('Index')
            h13 = subplot(7,1,3);
                        plot(TempKalmanMat(3,:))
                        title('MRP')
            h14 = subplot(7,1,4);
                        plot(TempKalmanMat(4,:))
                        title('Thumb F/E')
            h15 = subplot(7,1,5);
                        plot(TempKalmanMat(5,:))
                        title('Wrist F/E')
            h16 = subplot(7,1,6);
                        plot(TempKalmanMat(6,:))
                        title('Wrist RT')
            h17 = subplot(7,1,7);
                        plot(TempKalmanMat(7,:))
                        title('Bias Term')
            set([h11,h12,h13,h14,h15,h16,h17],'Ylim',[-1,1])
            linkaxes([h11,h12,h13,h14,h15,h16,h17],'x')
%             subplot(7,1,8)
%                         plot(KalmanMat')
%                         title ('All DOFs')
        end
        if showMvnt
           showMe(TempDEKAMat) 
        end
        if showMvnt || plotMvnt
            answerQ = questdlg('Keep Displayed Mvnt?','Review Mvnt','Keep','Discard','Exit');
        end
        
        %% need to append each trialsCell to DEKA_Mat and Kalman_Mat
        % to the DEKA_Mat(account for velocity control of wrists and Kalman_Mat (same as trialsCell)
        if strcmp(answerQ,'Keep')
            KalmanMat = [KalmanMat, cell2mat(trialsCell)];
            DEKAMat = KalmanMat;
            DEKAMat(PosVel',:) = [diff(DEKAMat(PosVel',:),1,2)*velFactor, restPos(PosVel')]; %%% velocity factor required to get the hand to move enough to see
        else
        end
        
    end
end
%% Plot each DOF sequence across all mvnts

%% SEND TO DEKA for Visual
    function showMe(DEKAMat)
        %% Structure to pass to the DEKA for mvnts
        % Init xippmex parameters and settings
        %     xl_close()
        % Connect to DEKA
        dk_toggle = 1;
        while dk_toggle ~= 10
            ms_deka_close();
            ms_deka_open();
            dk_toggle = double(ms_deka_mode());  %% why does it return 10 when the hand is off - Elliott?
            xl_pause(1)
        end
        % Training cycle loop
        mvntIdx = 1;
        t30 = double(xl_time);
        while mvntIdx<= size(DEKAMat,2)
            tcur = double(xl_time);
            if (tcur-t30) >= 990 % 990 is 33ms; 300 is 10ms
                t30 = tcur;
                % Send mvnts to DEKA
                posDEKA = Xhat2DEKA ([DEKAMat(1,mvntIdx),DEKAMat(2,mvntIdx),DEKAMat(3,mvntIdx),DEKAMat(4,mvntIdx),DEKAMat(5,mvntIdx),DEKAMat(6,mvntIdx)]);
                deka_move_COB(posDEKA);
                mvntIdx = mvntIdx+1;
            end
            
            if xl_event() %%% break from the DEMO if you want
                return
            end
        end
    end

%%% Save DEKAMat and KalmanMat to a .mat file 
% [] = uiputfile()
end
