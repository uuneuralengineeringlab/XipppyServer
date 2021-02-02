 function varargout = XipppyClientGUI(varargin)
% Last Modified by GUIDE v2.5 11-Sep-2020 08:54:51

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
set(handles.figure1,'Visible','off','CloseRequestFcn',@closeFigure)

% Opening log file
handles.LogFID = fopen('XipppyClientGUI.txt','w+');

% Initializing variables
handles.FName = 'XipppyClientGUI.mat';

% Initializing timing variables
handles.BaseLoopTime=0.033;
handles.MCalcTic = tic;
handles.MCalcTime = 0;
handles.MTotalTime = 0;
handles.LoopCnt = 1; %total loop iterations from start
handles.BaseCnt = 1;


% Initializing loop time plot
handles.MTimeBuff = zeros(300,2); %for plotting loop speed
handles.mH = plot(handles.MTimeAxH,handles.MTimeBuff);
set(handles.MTimeAxH,'xlim',[1,300],'ylim',[0,0.04],'xtick',[],'xticklabel',[],'ytick',[],'yticklabel',[],'box','on')

handles.PyTimeBuff = zeros(300,2); %for plotting loop speed
handles.pyH = plot(handles.PyTimeAxH,handles.PyTimeBuff);
set(handles.PyTimeAxH,'xlim',[1,300],'ylim',[0,50],'xtick',[],'xticklabel',[],'ytick',[],'yticklabel',[],'box','on')

% Initializing plot
handles.FeatBuff = zeros(300,32); %32 channels
handles.FeatH = plot(handles.FeatAxes,handles.FeatBuff);
set(handles.FeatAxes,'xlim',[1,300],'ylim',[-5000,5000],'xtick',[],'xticklabel',[],'box','on')
handles.CMap = flipud(jet(32));
for k=1:32
    set(handles.FeatH(k),'color',handles.CMap(k,:))
end

% Initializing timer
delete(timerfindall);
handles.timer1 = timer;
handles.timer1.ExecutionMode = 'fixedRate';
handles.timer1.Period = handles.BaseLoopTime;
handles.timer1.TasksToExecute = 1e12;
handles.timer1.BusyMode = 'drop';
handles.timer1.TimerFcn = {@mainLoop,handles.figure1};
handles.timer1.StopFcn = {@closeSystem,handles.figure1};

% Initializing arduino communication
handles.XC = XipppyClient; pause(1);

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

for k=1:32
    set(handles.FeatH(k),'ydata',handles.FeatBuff(:,k))
end
set(handles.etime,'string',num2str(handles.XC.Status.ElapsedTime,'%0.3f'))

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
% hObject    handle to StartTraining (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.XC.write('StartTraining')
