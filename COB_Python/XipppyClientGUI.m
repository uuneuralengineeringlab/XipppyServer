 function varargout = XipppyClientGUI(varargin)
% Last Modified by GUIDE v2.5 01-Mar-2021 12:53:48

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @XipppyClientGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @XipppyClientGUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before XipppyClientGUI is made visible.
function XipppyClientGUI_OpeningFcn(hObject, eventdata, handles, varargin)

% Setting figure properties
set(handles.figure1,'Visible','off','CloseRequestFcn',@closeFigure,'renderer','opengl')

% Opening log file
handles.LogFID = fopen('XipppyClientGUI.txt','w+');

% Initializing variables
handles.FName = 'XipppyClientGUI.mat';

% Initializing timing variables
handles.BaseLoopTime=0.066;
handles.MCalcTic = tic;
handles.MCalcTime = 0;
handles.MTotalTime = 0;
handles.LoopCnt = 1; %total loop iterations from start
handles.BaseCnt = 1;

set(handles.DOFTable,'data',zeros(6,3))
set(handles.StimTable,'data',repmat({[]},15,9))
set(handles.UserStimTable,'data',repmat({[]},15,5))
set(handles.BadElecTable,'data',repmat({[]},15,1))

% Initializing loop time plot
handles.MTimeBuff = zeros(300,2); %for plotting loop speed
handles.mH = plot(handles.MTimeAxH,handles.MTimeBuff);
set(handles.MTimeAxH,'xlim',[1,300],'ylim',[0,0.08],'xtick',[],'xticklabel',[],'ytick',[],'yticklabel',[],'box','on')

handles.PyTimeBuff = zeros(300,2); %for plotting loop speed
handles.pyH = plot(handles.PyTimeAxH,handles.PyTimeBuff);
set(handles.PyTimeAxH,'xlim',[1,300],'ylim',[0,50],'xtick',[],'xticklabel',[],'ytick',[],'yticklabel',[],'box','on')

% Initializing plot
handles.FeatBuff = zeros(300,48); %48 channels
handles.FeatH = plot(handles.FeatAxes,handles.FeatBuff);
set(handles.FeatAxes,'xlim',[1,300],'ylim',[-2000,2000],'xtick',[],'xticklabel',[],'box','on')
handles.CMap = flipud(jet(48));
for k=1:48
    set(handles.FeatH(k),'color',handles.CMap(k,:))
end

handles.KinBuff = zeros(300,6); 
handles.KinH = plot(handles.KinAxes,handles.KinBuff);
set(handles.KinAxes,'xlim',[1,300],'ylim',[-1.2,1.2],'xtick',[],'xticklabel',[],'box','on')
handles.KinCMap = flipud(jet(6));
for k=1:6
    set(handles.KinH(k),'color',handles.KinCMap(k,:))
end

handles.XhatBuff = zeros(300,6); 
handles.XhatH = plot(handles.XhatAxes,handles.XhatBuff);
set(handles.XhatAxes,'xlim',[1,300],'ylim',[-1.2,1.2],'xtick',[],'xticklabel',[],'box','on')
handles.XhatCMap = flipud(lines(6));
for k=1:6
    set(handles.XhatH(k),'color',handles.XhatCMap(k,:))
end

%1  = ind_lat (force)
%2  = ind_tip (force)
%3  = mid_tip (force)
%4  = ring_tip (force)
%5  = pinky_tip (force)
%6  = palm_dist (force)
%7  = palm_prox (force)
%8  = hand_edge (force)
%9  = hand_dorsal (force)
%10 = thumb_ulnar (force)
%11 = thumb_rad (force)
%12 = thumb_vol (force)
%13 = thumb_dor (force)
%14 = wrist_pron (position)
%15 = wrist_flex (position)
%16 = ind_flex (position)
%17 = mrp_flex (position)
%18 = thumb_pitch (position)
%19 = thumb_yaw (position)
handles.SensLookup = {'ind_lat','ind_tip','mid_tip','ring_tip','pinky_tip',...
    'palm_dist','palm_prox','hand_edge','hand_dorsal','thumb_ulnar',...
    'thumb_rad','thumb_vol','thumb_dor','wrist_pron','wrist_flex',...
    'ind_flex','mrp_flex','thumb_pitch','thumb_yaw'};
handles.SensBuff = zeros(300,19); 
handles.SensMask = false(19,1);
handles.SensMask(1:13) = true;
handles.SensH = plot(handles.SensAxes,handles.SensBuff);
set(handles.SensAxes,'xlim',[1,300],'ylim',[-1.2,1.2],'xtick',[],'xticklabel',[],'box','on')
handles.SensCMap = flipud(jet(19));
for k=1:19
    set(handles.SensH(k),'color',handles.SensCMap(k,:))
end

% Initializing timer
delete(timerfindall);
handles.timer1 = timer;
handles.timer1.ExecutionMode = 'fixedRate';
handles.timer1.Period = handles.BaseLoopTime;
handles.timer1.TasksToExecute = 1e12;
handles.timer1.BusyMode = 'queue';
handles.timer1.TimerFcn = {@mainLoop,handles.figure1};
handles.timer1.StopFcn = {@closeSystem,handles.figure1};

% Initializing arduino communication
% handles.XC = XipppyClient('localhost'); pause(1);
if nargin > 3
    handles.XC = XipppyClient(varargin{1});
else
    handles.XC = XipppyClient; 
end
% handles.figure1.Visible = 'on';
pause(1);

% while 1
%     if handles.XC.Connected
%         handles.ConnectedBtn.Value = true;
%         handles.ConnectedBtn.BackgroundColor = [0,1,0];
%         break;
%     end
%     pause(1);
% end
% 
% % send updated time to XipppyServer
% curTime = datestr(datetime('now'), 'dd mmm yyyy HH:MM:SS');
% cmdStr = ['TimeUpdate:', curTime];
% handles.XC.write(cmdStr);
% pause(1);

%get initial values from nomad

% Choose default command line output for XipppyClientGUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

start(handles.timer1);


% --- Outputs from this function are returned to the command line.
function varargout = XipppyClientGUI_OutputFcn(hObject, eventdata, handles) 
varargout{1} = handles.output;


%%%%%%%%%%%%%%%%%%%%%%%%% Timer Callbacks %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function mainLoop(varargin)
timer1 = varargin{1};
figure1 = varargin{3};
handles = guidata(figure1);
if handles.XC.Connected
    if ~handles.ConnectedBtn.Value
        handles.ConnectedBtn.Value = true;
        handles.ConnectedBtn.BackgroundColor = [0,1,0];
        % send updated time to XipppyServer
        curTime = datestr(datetime('now'), 'dd mmm yyyy HH:MM:SS');
        cmdStr = ['TimeUpdate:', curTime];
        handles.XC.write(cmdStr);
        pause(0.15);
        % request initial parameters from Nomad
%         handles = getNomadParams(handles);
    end
else
    if handles.ConnectedBtn.Value
        handles.ConnectedBtn.Value = false;
        handles.ConnectedBtn.BackgroundColor = [1,0,0];
    end
%     return;
end
try
    handles.MCalcTic = tic;
    handles = plotData(handles);
    handles.MCalcTime = toc(handles.MCalcTic); %calculation time within loop
    handles.MTotalTime = timer1.InstantPeriod; %overall loop time
    handles.LoopCnt = timer1.TasksExecuted;
catch ME
    assignin('base','ME',ME)
    fprintf('TGI: %s; name: %s; line: %0.0f\r\n',ME.message,ME.stack(1).name,ME.stack(1).line);
    stop(timer1);
end
guidata(figure1,handles);


function closeSystem(varargin)
figure1 = varargin{3};
handles = guidata(figure1);
handles.XC.close;
fclose(handles.LogFID);
delete(handles.timer1);
delete(handles.figure1);
handles = rmFieldsForSaving(handles);
handles = orderfields(handles);
assignin('base','handles',handles)
save(handles.FName,'handles');


function closeFigure(varargin)
figure1 = varargin{1};
handles = guidata(figure1);
if strcmp(handles.timer1.Running,'off')  
    handles.XC.close;
    fclose(handles.LogFID);
    delete(handles.timer1);
    delete(handles.figure1);
    handles = rmFieldsForSaving(handles);
    handles = orderfields(handles);
    assignin('base','handles',handles)
    save(handles.FName,'handles');
else
    stop(handles.timer1);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%% Local functions %%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handles = plotData(handles)
handles.FeatBuff(1:end-1,:) = handles.FeatBuff(2:end,:);
handles.FeatBuff(end,:) = handles.XC.Status.Features; 
for k=1:48
    set(handles.FeatH(k),'ydata',handles.FeatBuff(:,k))
end

handles.KinBuff(1:end-1,:) = handles.KinBuff(2:end,:);
handles.KinBuff(end,:) = handles.XC.Status.Kinematics; 
for k=1:6
    set(handles.KinH(k),'ydata',handles.KinBuff(:,k))
end

handles.XhatBuff(1:end-1,:) = handles.XhatBuff(2:end,:);
handles.XhatBuff(end,:) = handles.XC.Status.Xhat; 
for k=1:6
    set(handles.XhatH(k),'ydata',handles.XhatBuff(:,k))
end

handles.SensBuff(1:end-1,:) = handles.SensBuff(2:end,:);
handles.SensBuff(end,:) = handles.XC.Status.Sensors; 
for k=1:19
    if handles.SensMask(k)
        set(handles.SensH(k),'ydata',handles.SensBuff(:,k),'visible','on')
    else
        set(handles.SensH(k),'visible','off');
    end
end

handles.MTimeBuff(1:end-1,:) = handles.MTimeBuff(2:end,:);
handles.MTimeBuff(end,:) = [handles.MCalcTime,handles.MTotalTime]; 
set(handles.mH(1),'ydata',handles.MTimeBuff(:,1)) 
set(handles.mH(2),'ydata',handles.MTimeBuff(:,2)) 

handles.PyTimeBuff(1:end-1,:) = handles.PyTimeBuff(2:end,:);
handles.PyTimeBuff(end,:) = [handles.XC.Status.CalcTimePy,handles.XC.Status.ElapsedTimePy]; 
set(handles.pyH(1),'ydata',handles.PyTimeBuff(:,1)) 
set(handles.pyH(2),'ydata',handles.PyTimeBuff(:,2)) 



%Removing handles and timers from structure for saving
function handles = rmFieldsForSaving(handles)
hnames = fieldnames(handles);
for k=1:length(hnames)
    if ~isempty(handles.(hnames{k}))
        if isa(handles.(hnames{k}),'handle') || isa(handles.(hnames{k}),'timer')
            handles = rmfield(handles,hnames{k});
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%% Callbacks %%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% --- Executes on button press in StartTraining.
function StartTraining_Callback(hObject, eventdata, handles)
handles.XC.write('StartTraining')


% --- Executes on button press in LoadRecentWTS.
function LoadRecentWTS_Callback(hObject, eventdata, handles)
handles.XC.write('LoadRecentWTS')


% --- Executes on button press in LoadTrainingBtn.
function LoadTrainingBtn_Callback(hObject, eventdata, handles)
handles.XC.write('LoadTraining')


% --- Executes when entered data in editable cell(s) in DOFTable.
function DOFTable_CellEditCallback(hObject, eventdata, handles)
kin = hObject.Data(:,1)'; kin(7) = 0;
kinstr = regexprep(num2str(kin),'\s+',',');

lock = hObject.Data(:,2)'; lock(7) = 0;
lockstr = regexprep(num2str(lock),'\s+',',');

mirror = hObject.Data(:,3)'; mirror(7) = 0;
mirrorstr = regexprep(num2str(mirror),'\s+',',');

cmdstr = ['UpdateDOF:SS[''kin''] = np.array([',kinstr,'],dtype=np.single);' ...
    'SS[''lock_DOF''] = np.array([',lockstr,'],dtype=bool);' ...
    'SS[''mirror_DOF''] = np.array([',mirrorstr,'],dtype=int);'];
handles.XC.write(cmdstr)


% --- Executes on button press in ClearTblBtn.
function ClearTblBtn_Callback(hObject, eventdata, handles)
set(handles.StimTable,'data',repmat({[]},15,9))
set(handles.UserStimTable,'data',repmat({[]},15,5))
cmdstr = 'UpdateStimParams:SS[''stim_params''] = np.array([]).reshape((0,9))';
handles.XC.write(cmdstr)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on button press in SendParamsBtn.
function SendParamsBtn_Callback(hObject, eventdata, handles)
data = handles.StimTable.Data;
if all(reshape(cellfun(@isempty,data),[],1))
    cmdstr = 'UpdateStimParams:SS[''stim_params''] = np.array([]).reshape((0,9))';
else
    cmdstr = ['UpdateStimParams:SS[''stim_params''] = np.array([',sprintf('[%d,%d,%d,%d,%d,%d,%d,%d,%d],',int16(cell2mat(data))'),'])'];
end
handles.UserStimTable.Data = updateUserTable(data([data{:,8}] == 1, [1,2,4,5,9]),handles.SensLookup);
handles.XC.write(cmdstr)


% --- Executes on button press in GetStimParamsBtn.
function GetStimParamsBtn_Callback(hObject, eventdata, handles)
cmdstr = 'GetStimParams';
handles.XC.write(cmdstr)
while 1
    handles.XC.Event;
    if ~isempty(handles.XC.Event)
        break;
    end
    pause(0.1);
end
EventStr = regexp(handles.XC.Event,':','split','once');
% EventID = EventStr{1};
EventCmd = EventStr{2};
if regexp(EventCmd,'shape')
    set(handles.StimTable,'data',repmat({[]},15,9))
else
    eval(EventCmd);
    handles.StimTable.Data = [num2cell(data); repmat({[]},5,9)];
    handles.UserStimTable.Data = updateUserTable(num2cell(data(data(:,8)==1,[1,2,4,5,9])),handles.SensLookup);
end
handles.XC.Event = [];


% --- Executes on button press in GetUsrStim.
function GetUsrStim_Callback(hObject, eventdata, handles)
% hObject    handle to GetUsrStim (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles = getNomadParams(handles); % this should definitely not stay here..
cmdstr = 'GetStimParams';
handles.XC.write(cmdstr)
while 1
    handles.XC.Event;
    if ~isempty(handles.XC.Event)
        break;
    end
    pause(0.1);
end
EventStr = regexp(handles.XC.Event,':','split','once');
% EventID = EventStr{1};
EventCmd = EventStr{2};
if regexp(EventCmd,'shape')
    set(handles.StimTable,'data',repmat({[]},15,9))
else
    eval(EventCmd);
    handles.StimTable.Data = [num2cell(data); repmat({[]},5,9)];
    handles.UserStimTable.Data = updateUserTable(num2cell(data(data(:,8)==1,[1,2,4,5,9])),handles.SensLookup);
end
handles.XC.Event = [];


% --- Executes on button press in SendUsrStimBtn.
function SendUsrStimBtn_Callback(hObject, eventdata, handles)
% hObject    handle to SendUsrStimBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
data = handles.UserStimTable.Data;
if ~isempty(data)
    [~,idx] = ismember(data(:,2),handles.SensLookup);
    data(:,2) = num2cell(idx);
    if ~all(reshape(cellfun(@isempty,data),[],1))
        cmdstr = ['UpdateUserStimParams:SS[''stim_params''][np.ix_(SS[''stim_params''][:,7]==1, [3,4,8])] = np.array([',sprintf('[%d,%d,%d],',int16(cell2mat(data(:,3:5)))'),'])'];
    end
    handles.StimTable.Data([handles.StimTable.Data{:,8}] == 1, [1,2,4,5,9]) = data;
    handles.XC.write(cmdstr)
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% --- Executes on button press in CalStimTgl.
function CalStimTgl_Callback(hObject, eventdata, handles)
% hObject    handle to CalStimTgl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cmdstr = ['CalibrateStim:SS[''stop_hand''] = ', ...
    num2str(handles.CalStimTgl.Value)];
handles.StopHandBtn.Value = handles.CalStimTgl.Value;
handles.XC.write(cmdstr)


function dcell = updateUserTable(dcell,lut)
dcell(:,2) = lut([dcell{:,2}]);


% --- Executes on button press in SendBadElecs.
function SendBadElecs_Callback(hObject, eventdata, handles)
% hObject    handle to SendBadElecs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
data = handles.BadElecTable.Data;
nonNan = cell2mat(cellfun(@(x) ~isnan(x), data, 'UniformOutput', false));
data = data(nonNan);
if all(reshape(cellfun(@isempty,data),[],1))
    cmdstr = 'UpdateBadElecs:SS[''bad_EMG_elecs''] = np.array([], dtype=int).reshape((0,1))';
else
    cmdstr = ['UpdateBadElecs:SS[''bad_EMG_elecs''] = np.array([',sprintf('%d, ',int16(cell2mat(data))'),'], dtype=int)'];
end
handles.XC.write(cmdstr)




% --- Executes on button press in GetBadElecs.
function GetBadElecs_Callback(hObject, eventdata, handles)
% hObject    handle to GetBadElecs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cmdstr = 'GetBadElecs';
handles.XC.write(cmdstr)
while 1
    handles.XC.Event;
    if ~isempty(handles.XC.Event)
        break;
    end
    pause(0.1);
end
EventStr = regexp(handles.XC.Event,':','split','once');
% EventID = EventStr{1};
EventCmd = EventStr{2};
if regexp(EventCmd,'shape')
    set(handles.BadElecTable,'data',repmat({[]},15,1))
else
    eval(EventCmd);
    if isempty(data)
        set(handles.BadElecTable,'data',repmat({[]},15,1))
    else
        handles.BadElecTable.Data = [num2cell(data)'; repmat({[]},5,1)];
    end
end

handles.XC.Event = [];


% --- Executes on button press in CloseXSBtn.
function CloseXSBtn_Callback(hObject, eventdata, handles)
% hObject    handle to CloseXSBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cmdstr = 'close';
handles.XC.write(cmdstr)


% --- Executes on button press in StopStimBtn.
function StopStimBtn_Callback(hObject, eventdata, handles)
% hObject    handle to StopStimBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cmdstr = ['StopStim:SS[''stop_stim''] = ', ...
    num2str(handles.StopStimBtn.Value)];
handles.XC.write(cmdstr)


% --- Executes on button press in StopHandBtn.
function StopHandBtn_Callback(hObject, eventdata, handles)
% hObject    handle to StopHandBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

cmdstr = ['StopHand:SS[''stop_hand''] = ', ...
    num2str(handles.StopHandBtn.Value)];
handles.CalStimTgl.Value = handles.StopHandBtn.Value;
handles.XC.write(cmdstr)


function handles = getNomadParams(handles)
% handles    structure with handles and user data (see GUIDATA)

% get all the parameters needed to update GUI
cmdstr = 'GetNomadParams';
handles.XC.write(cmdstr)
while 1
    handles.XC.Event;
    if ~isempty(handles.XC.Event)
        break;
    end
    pause(0.1);
end
EventStr = regexp(handles.XC.Event,':','split','once');
% EventID = EventStr{1};
EventCmd = EventStr{2};
if regexp(EventCmd,'shape')
    set(handles.BadElecTable,'data',repmat({[]},15,1))
else
    eval(EventCmd); % this populates the fields in the following lines.
    % EMG electrodes
    if isempty(bad_EMG_elecs)
        set(handles.BadElecTable,'data',repmat({[]},15,1))
    else
        handles.BadElecTable.Data = [num2cell(bad_EMG_elecs)'; repmat({[]},5,1)];
    end
    % kin
    handles.DOFTable.Data(:,1) = kin;
    % lock_DOF 
    handles.DOFTable.Data(:,2) = lock_DOF(1:6);
    % mirror_DOF
    handles.DOFTable.Data(:,3) = mirror_DOF(1:6);
    % stim params
    if any(any(stim_params))
        handles.StimTable.Data = [num2cell(stim_params); repmat({[]},5,9)];
        handles.UserStimTable.Data = updateUserTable(num2cell(stim_params(stim_params(:,8)==1,[1,2,4,5,9])),handles.SensLookup);
    else
        set(handles.StimTable,'data',repmat({[]},15,9))
        set(handles.UserStimTable,'data',repmat({[]},15,5))
    end
    % stop_hand
    handles.StopHandBtn.Value = stop_hand;
    handles.CalStimTgl.Value = stop_hand;
    % stop stim
    handles.StopStimBtn.Value = stop_stim;
    
    
end

handles.XC.Event = [];
