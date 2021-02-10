% Initialization & callbacks of GUI for mouse turntable experiment
% Written for Salk Institute, Azim lab
% Written by Masakazu Igarashi, Mark Stambaugh & others
% Last edited 2020/03/13

function varargout = TactileOrientationTask(varargin)
% TACTILEORIENTATIONTASK MATLAB code for TactileOrientationTask.fig
%      TACTILEORIENTATIONTASK, by itself, creates a new TACTILEORIENTATIONTASK or raises the existing
%      singleton*.
%
%      H = TACTILEORIENTATIONTASK returns the handle to a new TACTILEORIENTATIONTASK or the handle to
%      the existing singleton*.
%
%      TACTILEORIENTATIONTASK('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TACTILEORIENTATIONTASK.M with the given input arguments.
%
%      TACTILEORIENTATIONTASK('Property','Value',...) creates a new TACTILEORIENTATIONTASK or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before TactileOrientationTask_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to TactileOrientationTask_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help TactileOrientationTask

% Last Modified by GUIDE v2.5 14-Mar-2020 22:44:41

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @TactileOrientationTask_OpeningFcn, ...
    'gui_OutputFcn',  @TactileOrientationTask_OutputFcn, ...
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


% --- Executes just before TactileOrientationTask is made visible.
function TactileOrientationTask_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to TactileOrientationTask (see VARARGIN)

% Choose default command line output for TactileOrientationTask
handles.system_status=zeros(1,10);% 1: daq search, 2: daq connect
handles.output = hObject;
handles.Difficulty_matrix.ColumnEditable=true(1,6);
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes TactileOrientationTask wait for user response (see UIRESUME)
% uiwait(handles.figure1);

% --- Executes during object deletion, before destroying properties.
function figure1_DeleteFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
instrreset();



% --- Outputs from this function are returned to the command line.
function varargout = TactileOrientationTask_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in read_daq1.
function Scan_button_Callback(hObject, eventdata, handles)
% hObject    handle to read_daq1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.Connect_button,'enable', 'off');
set(handles.Initialize_button,'enable', 'off');
set(handles.Start_button,'enable', 'off');
set(handles.End_button,'enable', 'off');

devices = search_for_devices("rotary table", 115200);
set(handles.DeviceList,'string',devices);
if size(devices) > 0
    set(handles.Connect_button,'enable', 'on');
end


% --- Executes on button press in connect_daq.
function Connect_button_Callback(hObject, eventdata, handles)
% hObject    handle to connect_daq (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Device_contents = cellstr(get(handles.DeviceList,'String'));
selected = get(handles.DeviceList,'Value');
SelectedDevice = Device_contents{selected};

% open port and add to handles object so other functions can access it
port_handle = serial(SelectedDevice, "BaudRate", 115200);
fopen(port_handle);

handles.port = port_handle;
guidata(hObject, handles);

pause(2);

%print startup text to the terminal
while port_handle.BytesAvailable
    fprintf(fscanf(port_handle));
end

set(handles.Initialize_button,'enable', 'on');
set(handles.test_hardware_button,'enable', 'on');

% --- Executes on button press in Initialize.
function Initialize_button_Callback(hObject, eventdata, handles)
% hObject    handle to Initialize (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA) 

params.zero_position_angle_deg = floor(str2double(get(handles.zero_angle_deg,'string'))); 
params.trial_duration_ms = floor(str2double(get(handles.duration_ms,'string')));
params.preamble_ms = floor(str2double(get(handles.preamble_ms,'string')));
params.postamble_ms = floor(str2double(get(handles.postamble_ms,'string')));
params.reward_anyway = floor(get(handles.reward_always,'value'));
params.reward_duration_ms = floor(str2double(get(handles.reward_duration_ms,'string')));
params.data_res_ms = floor(str2double(get(handles.sample_rate_ms,'string')));
params.opto_pre_trial_ms = floor(str2double(get(handles.opto_pre_trial_ms,'string')));
params.opto_post_trial_ms = floor(str2double(get(handles.opto_post_trial_ms,'string')));

send_parameters(handles.port, params);

preamble_s = 0.001 * params.preamble_ms;
postamble_s = 0.001 * params.postamble_ms;
trial_duration_s = 0.001 * params.trial_duration_ms;
total_length_s = preamble_s + postamble_s + trial_duration_s;


% initialize figure visualization
handles.axes1;
hold off;
plot(0,0);
hold on;
box off;
set(handles.axes1,'Color','white', 'FontSize', 10, 'LineWidth', 0.4);
xlabel('duration (sec)');
ylabel('displacement (deg)');
xlim([0, total_length_s]);
ylim([-31, 160]);

guidata(hObject, handles);

set(handles.Start_button,'enable', 'on');


% --- Executes on button press in start.
function Start_button_Callback(hObject, eventdata, handles)
% hObject    handle to start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% check inputs for valid update parameters
if get(handles.Difficulty_update, "Value")
    if str2double(get(handles.Min_success, "String")) > str2double(get(handles.Max_success, "String"))
        msgbox("the Maximum success has to be larger than Mimimum success");
        error("parameter error");
    end
end

% check for valid opto parameters
if get(handles.opto_ON, 'Value')
    if str2double(get(handles.opto_pre_trial_ms, "String")) < str2double(get(handles.preamble_ms,"String"))
        msgbox("the preamble must be larger than the opto pre-trial");
        error("parameter error");
    end
    if str2double(get(handles.opto_pre_trial_ms, "String")) < str2double(get(handles.preamble_ms,"String"))
        msgbox("the postamble must be larger than the opto post-trial");
        error("parameter error");
    end
    if str2double(get(handles.warm_up_trials, "String")) > str2double(get(handles.max_trials,"String"))
        msgbox("the total trials must be larger than the opto warm-up trials");
        error("parameter error");
    end
end

opto_warm_up_trials = floor(str2double(get(handles.warm_up_trials,"String")));
opto_ratio = floor(str2double(get(handles.opto_ratio,"String")));

%setting saving directory------------------------------------
animalID = get(handles.animalID,"String");
currentDir = get(handles.WorkDir,"String");
DateTime=datestr(now,'yyyymmddTHHMMSS');
destdirectory = strcat(currentDir,"\",DateTime(1:8),"_",animalID);
if exist(destdirectory, 'dir')
    answer = questdlg('You already have same animal ID in the folder. Are you sure to overwrite?', ...
        'Warning', ...
        'Yes','No',"No");
    switch answer
        case 'Yes'
            mkdir(char(destdirectory));   %create the directory
        case 'No'
            error('Breaking out');
    end
end
mkdir(char(destdirectory));   %create the directory

%create master storage matrix-----------------------------------------------------------
Master_data = cell(10+1,5);
Master_data{1,1}='timeStamp';
Master_data{1,2}='turnTableAngle(deg)';
Master_data{1,3}='Success/Fail';
Master_data{1,4}='TaskParameters';
Master_data{1,5}='Opto'; 

% import difficulty settings from GUI
Difficulty = get(handles.Difficulty_matrix,"Data");
Level_count = length(Difficulty);
Level = 1;

target_angle_deg = Difficulty(1,Level);
window_play_deg = Difficulty(2,Level);
RandomRange_deg = Difficulty(3,Level);
hold_time_ms = Difficulty(4,Level);

params.target_angle_deg = target_angle_deg;
params.window_play_deg = window_play_deg;
params.hold_time_ms = hold_time_ms;

send_parameters(handles.port, params);
params = rmfield(params,fieldnames(params)); %empty params

%ZeroPosition_deg = str2double(get(handles.zero_angle_deg,'string'));

% initialize plot with target +/- threshold window
Min_Threshold = target_angle_deg - window_play_deg;
Max_Threshold = target_angle_deg + window_play_deg;

preamble_s = 0.001 * floor(str2double(get(handles.preamble_ms,'string')));
trial_duration_s = 0.001 * floor(str2double(get(handles.duration_ms,'string')));

handles.axes1;
rectangle('Position',[preamble_s, Min_Threshold, trial_duration_s, Max_Threshold-Min_Threshold], ...
        'FaceColor',[1 .5 .5 0.2], 'EdgeColor', [1 1 1 0]);
title(strcat(DateTime(1:8),"-",animalID));
xlabel('duration (sec)');ylabel('displacement (deg)');

% trial structure---------------
max_trials = str2double(get(handles.Max_trials,"String"));
trials_per_segment = round(str2double(get(handles.trial_interval,"String")));
max_successes = round(str2double(get(handles.Max_NumSuc,"String")));
level_up_success_rate = str2double(get(handles.Max_success,"String"));
level_down_success_rate = str2double(get(handles.Min_success,"String"));
update_difficulty_bool = get(handles.Difficulty_update,"Value");

preamble_s = 0.001 * str2double(get(handles.preamble_ms,"String"));
postamble_s = 0.001 * str2double(get(handles.postamble_ms,"String"));
trialDuration_s = 0.001*str2double(get(handles.duration_ms,'string'));
total_length_s = preamble_s + postamble_s + trialDuration_s;
sample_rate_Hz = ceil(1000/(str2double(get(handles.sample_rate_ms,'string'))));

%initialize loop  control variables
trials = 1; 
successes = 0; 
trials_this_segment = 1;
successes_this_segment = 0;

set(handles.End_button,'enable', 'on');
setappdata(0,"end_now", 0);
tic;
% continue until enough successes or trial limit reached or END pushed
while (successes < max_successes) && (trials <= max_trials) && (~getappdata(0,"end_now"))
    % update the gui with current values
    set(handles.Suc_rate_monitor,"String",num2str(successes/trials));
    set(handles.Current_Level,"String",num2str(Level));
    set(handles.Total_reward,"String",num2str(successes));
    set(handles.Current_trial,"String",num2str(trials));
    
    % program offset for next trial
    start_angle_deg = randi([-RandomRange_deg RandomRange_deg]);
    params.start_angle_deg = start_angle_deg; 
    
    % determine if opto will be used in this trial
    
    opto_enabled = get(handles.opto_ON,'value') && (trials > opto_warm_up_trials) && (rand() <= opto_ratio);
    
    params.opto_enabled = opto_enabled;
    
    pause(1); %wait for arduino to get back to command tree
    send_parameters(handles.port, params);
    params = rmfield(params, fieldnames(params)); %empty params
    
    
    % initialize time & angle arrays.
    time_s = nan(total_length_s*sample_rate_Hz, 1);
    angle_deg = nan(total_length_s*sample_rate_Hz, 1);
    trial_success = nan;
    
    % signal Arduino to start trial
    fprintf(handles.port, 'r');
    while (~handles.port.BytesAvailable)
    end
    
    msg = fscanf(handles.port);
    if contains(msg, "start")
        fprintf("beginning trial\n");
    else
        fprintf("error!");
        error("trial not started");
    end
    
    opto_on_color = [1, 0, 0, 0.4];
    opto_off_color = [0, 0.5, 1, 0.4];
    
    % read trial data until the Arduino transmits end of trial
    i = 0;
    last_drawn_index = 1;
    draw_interval_s = 0.1;
    line_color = opto_off_color;
    tic;
    trial_end = 0;
    while (~trial_end)
        if handles.port.BytesAvailable
            msg = fscanf(handles.port);
            if contains(msg, "success")
                trial_success = 1;
                scatter(time_s(i), angle_deg(i), 10,'MarkerEdgeAlpha',0,'MarkerFaceColor', 'b','MarkerFaceAlpha', 0.5);
            elseif contains(msg, "failure")
                trial_success = 0;
                scatter(time_s(i), angle_deg(i), 10,'MarkerEdgeAlpha',0,'MarkerFaceColor', 'b','MarkerFaceAlpha', 0.5);
            elseif contains(msg, "end")
                trial_end = 1;
            elseif contains(msg, "on")
                tic;
                plot(time_s(last_drawn_index:i), angle_deg(last_drawn_index:i),'Color',line_color,'LineWidth', 1.2); 
                drawnow; %interrupt opportunity for END callback
                last_drawn_index = i;
                line_color = opto_on_color;
            elseif contains(msg, "off")
                tic;
                plot(time_s(last_drawn_index:i), angle_deg(last_drawn_index:i),'Color',line_color,'LineWidth', 1.2); 
                drawnow; %interrupt opportunity for END callback
                line_color = opto_off_color;
            else
                time_ms_angle_deg = textscan(msg, "%f %f");
                if length(time_ms_angle_deg) == 2
                    i = i+1;
                    time_s(i) = cell2mat(time_ms_angle_deg(1)) * 0.001;
                    angle_deg(i) = cell2mat(time_ms_angle_deg(2));
                    % angle_deg(i) = angle_deg(i) + 0.5*i + 10*trials; 
                    if toc >= draw_interval_s
                        tic;
                        plot(time_s(last_drawn_index:i), angle_deg(last_drawn_index:i),'Color',line_color,'LineWidth', 1.2); 
                        drawnow; %interrupt opportunity for END callback
                        last_drawn_index = i;
                    end
                end
            end
        end
    end
    %TODO when parameters are edited, disable appropriate buttons
    %draw remaining datapoints
    plot(time_s(last_drawn_index:i), angle_deg(last_drawn_index:i),'Color',[0, 0.5, 1, 0.4],'LineWidth', 1.2);
    drawnow; %interrupt opportunity for END callback
    
    successes = successes + trial_success;
    successes_this_segment = successes_this_segment + trial_success;
    
    % truncate data based on actual run length
    time_s = time_s(1:i);
    angle_deg = angle_deg(1:i);
    
    % clear the serial buffer
    while handles.port.BytesAvailable
        fscanf(handles.port);
    end
    
    %save individual trial data
    parameters = strcat("startAngle= ", num2str(start_angle_deg), ...
                        " trialDuration= ", num2str(trialDuration_s), ...
                        " holdTime(ms)= ", num2str(hold_time_ms), ...
                        " TargetAngle= ", num2str(target_angle_deg), ...
                        " WindowPlay= ", num2str(window_play_deg), ...
                        " RandomRange= ", num2str(RandomRange_deg), ...
                        " Opto_pre_trial_ms", get(handles.opto_pre_trial_ms,"String"), ...
                        " Opto_post_trial_ms", get(handles.opto_post_trial_ms,"String") ...
                        );
                    
    filename = char(strcat('trial_',num2str(trials)));
    save(char(strcat(destdirectory,'\',filename)),'time_s', 'angle_deg', 'trial_success', 'parameters', 'opto_enabled');
    
    
    %copy to master data array
    Master_data{trials+1,1} = time_s;
    Master_data{trials+1,2} = angle_deg;
    Master_data{trials+1,3} = trial_success;
    Master_data{trials+1,4} = parameters;
    Master_data{trials+1,5} = opto_enabled;
    
    % check if END button has been pushed
    if getappdata(0,"end_now")
        break;
    end
    
    % Check if at end of segment. If so, adjust level and update parameters
    if trials_this_segment == trials_per_segment 
        if update_difficulty_bool
            SucRate = successes_this_segment / trials_per_segment; % successful rate
            if SucRate >= level_up_success_rate
                Level = min(Level + 1, Level_count);
            elseif SucRate < level_down_success_rate
                Level = max(Level - 1, 1);
            end
        else
            Level = min(Level + 1, Level_count);
        end
        
        %read new parameters and send to the Arduino
        target_angle_deg = Difficulty(1,Level);
        window_play_deg = Difficulty(2,Level);
        RandomRange_deg = Difficulty(3,Level);
        hold_time_ms = Difficulty(4,Level);% in ms between 10-2000m
        
        params.target_angle_deg = target_angle_deg;
        params.window_play_deg = window_play_deg;
        params.hold_time_ms = hold_time_ms;
        
        send_parameters(handles.port, params);
        params = rmfield(params,fieldnames(params)); %empty params
        
        % update with new target +/- threshold window
        Min_Threshold = target_angle_deg - window_play_deg;
        Max_Threshold = target_angle_deg + window_play_deg;

        handles.axes1;
        rectangle('Position',[preamble_s, Min_Threshold, trialDuration_s, Max_Threshold-Min_Threshold], ...
                'FaceColor',[1 .5 .5 0.2], 'EdgeColor', [1 1 1 0]);
        title(strcat(DateTime(1:8),"-",animalID));
        xlabel('duration (sec)');ylabel('displacement (deg)');
        
        trials_this_segment = 1;
        successes_this_segment = 0;
    else
        trials_this_segment = trials_this_segment + 1;
    end
    %TODO verify trial window success
    trials = trials + 1;
end
% trials complete or END pressed
fprintf("trials complete\n");

% save master data
filename = "master_data";
save(char(strcat(destdirectory,'\',filename)),'Master_data');

% copy final figure and save with the data
fignew = figure('Visible','off','Position',[500 500 400 250]); % Invisible figure

newAxes = copyobj(handles.axes1,fignew); % Copy the appropriate axes
set(newAxes,'Position',get(groot,'DefaultAxesPosition')); % The original position is copied too, so adjust it.
set(fignew,'CreateFcn','set(gcbf,''Visible'',''on'')'); % Make it visible upon loading

saveas(fignew,char(strcat(destdirectory,'\','result.fig')))
delete(fignew);
answer = questdlg('Task completed. Save settings?', ...
        'Warning', ...
        'yes','No',"No");
switch answer
    case 'yes'
        save_setting_Callback(hObject, eventdata, handles)
    case 'No'
end


% --- Executes on button press in END button.
function End_button_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
setappdata(0, "end_now", 1);
set(handles.End_button,'enable', 'off');
set(handles.Start_button,'enable', 'off');
disp("ending");

% --- Executes on button press in save_setting.
function save_setting_Callback(hObject, eventdata, handles)
% hObject    handle to save_setting (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
selpath = uigetdir(get(handles.WorkDir,"String"));
animalID = get(handles.animalID,"String");
DateTime = datestr(now,'yyyymmddTHHMMSS');

state.WorkDir = get(handles.WorkDir,"String");
state.Difficulty_matrix = get(handles.Difficulty_matrix,"Data");
state.animalID = get(handles.animalID,"String");
state.zero_angle_deg = get(handles.zero_angle_deg,'string');

state.Difficulty_update = get(handles.Difficulty_update,"Value");
state.Min_success = get(handles.Min_success,"String");
state.Max_success = get(handles.Max_success,"String");

state.trial_interval = get(handles.trial_interval,"String");
state.Max_trials = get(handles.Max_trials,"String");
state.Max_NumSuc = get(handles.Max_NumSuc,"String");

state.sample_rate_ms = get(handles.sample_rate_ms,'string');
state.preamble_ms = get(handles.preamble_ms,'string');
state.duration_ms = get(handles.duration_ms,'string');
state.postamble_ms = get(handles.postamble_ms,'string');

state.reward_always = get(handles.reward_always,'value');
state.reward_duration_ms = get(handles.reward_duration_ms,'string');

state.opto_on = get(handles.opto_ON,'value');
state.opto_warm_up_trials = get(handles.warm_up_trials,"String");
state.opto_ratio = get(handles.opto_ratio,"String");
state.opto_pre_trial_ms = get(handles.opto_pre_trial_ms,"String");
state.opto_post_trial_ms = get(handles.opto_post_trial_ms,"String");


di = char(strcat(selpath,"\","task_settings",DateTime(1:8),"_",animalID,".mat"));
if exist(di, 'file')==2
    answer = questdlg('You already have same date with same animal ID in the folder. Are you sure to overwrite?', ...
        'Warning', ...
        'yes','No',"No");
    if answer=='yes'
        save(di,"state");
    end
else
    save(di,"state");
end

% --- Executes on button press in load_setting.
function load_setting_Callback(hObject, eventdata, handles)
% hObject    handle to load_setting (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[file,path] = uigetfile(get(handles.WorkDir,"String"));
load(char(strcat(path,file)));
if exist("state","var")==0
    msgbox("wrong file")
    error("a wrong file has opened")
end

set(handles.WorkDir, "String", state.WorkDir);
set(handles.Difficulty_matrix, "Data", state.Difficulty_matrix);
set(handles.animalID, "String", state.animalID);
set(handles.zero_angle_deg, 'string', state.zero_angle_deg);

set(handles.Difficulty_update,"Value", state.Difficulty_update);
set(handles.Min_success,"String", state.Min_success);
set(handles.Max_success,"String", state.Max_success);

set(handles.trial_interval, "String", state.trial_interval);
set(handles.Max_trials, "String", state.Max_trials);
set(handles.Max_NumSuc, "String", state.Max_NumSuc);

set(handles.sample_rate_ms, 'string', state.sample_rate_ms);
set(handles.preamble_ms, 'string', state.preamble_ms);
set(handles.duration_ms, 'string', state.duration_ms);
set(handles.postamble_ms, 'string', state.postamble_ms);

set(handles.reward_always, 'value', state.reward_always);
set(handles.reward_duration_ms, 'string', state.reward_duration_ms);

set(handles.opto_ON, 'value', state.opto_on);
set(handles.warm_up_trials, "String", state.opto_warm_up_trials);
set(handles.opto_ratio, "String", state.opto_ratio);
set(handles.opto_pre_trial_ms, "String", state.opto_pre_trial_ms);
set(handles.opto_post_trial_ms, "String", state.opto_post_trial_ms);


% --- Executes on button press in pushbutton8.
function keyboard_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)\
fprintf("keyboard enabled\n");
keyboard % Why is this here? --MPS 2020-02-27


% -------------------------------------------------------------------------
% ALL UNUSED FUNCTIONS BELOW HERE
% -------------------------------------------------------------------------


% --- Executes on selection change in DeviceList.
function DeviceList_Callback(hObject, eventdata, handles)
% hObject    handle to DeviceList (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: contents = cellstr(get(hObject,'String')) returns DeviceList contents as cell array
%        contents{get(hObject,'Value')} returns selected item from DeviceList

% --- Executes during object creation, after setting all properties.
function DeviceList_CreateFcn(hObject, eventdata, handles)
% hObject    handle to DeviceList (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double

% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double

% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Max_trials_Callback(hObject, eventdata, handles)
% hObject    handle to Max_trials (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of Max_trials as text
%        str2double(get(hObject,'String')) returns contents of Max_trials as a double
val = floor(str2double(get(hObject, 'String')));
val = max(val, 1);
set(hObject, 'String', val);

% --- Executes during object creation, after setting all properties.
function Max_trials_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Max_trials (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double

% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

function animalID_Callback(hObject, eventdata, handles)
% hObject    handle to animalID (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of animalID as text
%        str2double(get(hObject,'String')) returns contents of animalID as a double

% --- Executes during object creation, after setting all properties.
function animalID_CreateFcn(hObject, eventdata, handles)
% hObject    handle to animalID (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in Difficulty_update.
function Difficulty_update_Callback(hObject, eventdata, handles)
% hObject    handle to Difficulty_update (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of Difficulty_update

function Min_success_Callback(hObject, eventdata, handles)
% hObject    handle to Min_success (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of Min_success as text
%        str2double(get(hObject,'String')) returns contents of Min_success as a double
val = str2double(get(hObject, 'String'));
val = max(val, 0.1);
val = min(val, 0.9);
set(hObject, 'String', val);


% --- Executes during object creation, after setting all properties.
function Min_success_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Min_success (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Max_success_Callback(hObject, eventdata, handles)
% hObject    handle to Max_success (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of Max_success as text
%        str2double(get(hObject,'String')) returns contents of Max_success as a double
val = str2double(get(hObject, 'String'));
val = max(val, 0.1);
val = min(val, 0.9);
set(hObject, 'String', val);


% --- Executes during object creation, after setting all properties.
function Max_success_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Max_success (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function trial_interval_Callback(hObject, eventdata, handles)
% hObject    handle to trial_interval (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of trial_interval as text
%        str2double(get(hObject,'String')) returns contents of trial_interval as a double
val = floor(str2double(get(hObject, 'String')));
val = max(val, 5);
val = min(val, 20);
set(hObject, 'String', val);


% --- Executes during object creation, after setting all properties.
function trial_interval_CreateFcn(hObject, eventdata, handles)
% hObject    handle to trial_interval (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: place code in OpeningFcn to populate axes1

function WorkDir_Callback(hObject, eventdata, handles)
% hObject    handle to WorkDir (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of WorkDir as text
%        str2double(get(hObject,'String')) returns contents of WorkDir as a double

% --- Executes during object creation, after setting all properties.
function WorkDir_CreateFcn(hObject, eventdata, handles)
% hObject    handle to WorkDir (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
set(hObject, 'String', pwd()); % set to present working directory

% --- Executes on button press in opto_ON.
function opto_ON_Callback(hObject, eventdata, handles)
% hObject    handle to opto_ON (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of opto_ON

function Max_NumSuc_Callback(hObject, eventdata, handles)
% hObject    handle to Max_NumSuc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of Max_NumSuc as text
%        str2double(get(hObject,'String')) returns contents of Max_NumSuc as a double
val = floor(str2double(get(hObject, 'String')));
val = max(val, 1);
set(hObject, 'String', val);

% --- Executes during object creation, after setting all properties.
function Max_NumSuc_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Max_NumSuc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Suc_rate_monitor_Callback(hObject, eventdata, handles)
% hObject    handle to Suc_rate_monitor (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of Suc_rate_monitor as text
%        str2double(get(hObject,'String')) returns contents of Suc_rate_monitor as a double

% --- Executes during object creation, after setting all properties.
function Suc_rate_monitor_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Suc_rate_monitor (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Current_Level_Callback(hObject, eventdata, handles)
% hObject    handle to Current_Level (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of Current_Level as text
%        str2double(get(hObject,'String')) returns contents of Current_Level as a double

% --- Executes during object creation, after setting all properties.
function Current_Level_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Current_Level (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Total_reward_Callback(hObject, eventdata, handles)
% hObject    handle to tes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of tes as text
%        str2double(get(hObject,'String')) returns contents of tes as a double

% --- Executes during object creation, after setting all properties.
function tes_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Current_trial_Callback(hObject, eventdata, handles)
% hObject    handle to Current_trial (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of Current_trial as text
%        str2double(get(hObject,'String')) returns contents of Current_trial as a double

% --- Executes during object creation, after setting all properties.
function Current_trial_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Current_trial (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function Total_reward_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Total_reward (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in reward_always.
function reward_always_Callback(hObject, eventdata, handles)
% hObject    handle to reward_always (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of reward_always

function warm_up_trials_Callback(hObject, eventdata, handles)
% hObject    handle to warm_up_trials (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of warm_up_trials as text
%        str2double(get(hObject,'String')) returns contents of warm_up_trials as a double
val = floor(str2double(get(hObject, 'String')));
val = max(val, 1);
set(hObject, 'String', val);

% --- Executes during object creation, after setting all properties.
function warm_up_trials_CreateFcn(hObject, eventdata, handles)
% hObject    handle to warm_up_trials (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function opto_ratio_Callback(hObject, eventdata, handles)
% hObject    handle to opto_ratio (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of opto_ratio as text
%        str2double(get(hObject,'String')) returns contents of opto_ratio as a double
val = str2double(get(hObject, 'String'));
val = max(val, 0);
val = min(val, 1);
set(hObject, 'String', val);

% --- Executes during object creation, after setting all properties.
function opto_ratio_CreateFcn(hObject, eventdata, handles)
% hObject    handle to opto_ratio (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in invoke_keyboard.
function invoke_keyboard_Callback(hObject, eventdata, handles)
% hObject    handle to invoke_keyboard (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function zero_angle_deg_Callback(hObject, eventdata, handles)
% hObject    handle to zero_angle_deg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of zero_angle_deg as text
%        str2double(get(hObject,'String')) returns contents of zero_angle_deg as a double
val = floor(str2double(get(hObject, 'String')));
val = max(val, 0);
val = min(val, 180);
set(hObject, 'String', val);

% --- Executes during object creation, after setting all properties.
function zero_angle_deg_CreateFcn(hObject, eventdata, handles)
% hObject    handle to zero_angle_deg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function sample_rate_ms_Callback(hObject, eventdata, handles)
% hObject    handle to sample_rate_ms (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of sample_rate_ms as text
%        str2double(get(hObject,'String')) returns contents of sample_rate_ms as a double
val = floor(str2double(get(hObject, 'String')));
val = max(val, 1);
set(hObject, 'String', val);

% --- Executes during object creation, after setting all properties.
function sample_rate_ms_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sample_rate_ms (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function opto_pre_trial_ms_Callback(hObject, eventdata, handles)
% hObject    handle to opto_pre_trial_ms (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of opto_pre_trial_ms as text
%        str2double(get(hObject,'String')) returns contents of opto_pre_trial_ms as a double
val = floor(str2double(get(hObject, 'String')));
val = max(val, 1);
set(hObject, 'String', val);

% --- Executes during object creation, after setting all properties.
function opto_pre_trial_ms_CreateFcn(hObject, eventdata, handles)
% hObject    handle to opto_pre_trial_ms (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function opto_post_trial_ms_Callback(hObject, eventdata, handles)
% hObject    handle to opto_post_trial_ms (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of opto_post_trial_ms as text
%        str2double(get(hObject,'String')) returns contents of opto_post_trial_ms as a double
val = floor(str2double(get(hObject, 'String')));
val = max(val, 1);
set(hObject, 'String', val);

% --- Executes during object creation, after setting all properties.
function opto_post_trial_ms_CreateFcn(hObject, eventdata, handles)
% hObject    handle to opto_post_trial_ms (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function postamble_ms_Callback(hObject, eventdata, handles)
% hObject    handle to postamble_ms (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of postamble_ms as text
%        str2double(get(hObject,'String')) returns contents of postamble_ms as a double
val = floor(str2double(get(hObject, 'String')));
val = max(val, 1);
set(hObject, 'String', val);

% --- Executes during object creation, after setting all properties.
function postamble_ms_CreateFcn(hObject, eventdata, handles)
% hObject    handle to postamble_ms (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function preamble_ms_Callback(hObject, eventdata, handles)
% hObject    handle to preamble_ms (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of preamble_ms as text
%        str2double(get(hObject,'String')) returns contents of preamble_ms as a double
val = floor(str2double(get(hObject, 'String')));
val = max(val, 0);
set(hObject, 'String', val);

% --- Executes during object creation, after setting all properties.
function preamble_ms_CreateFcn(hObject, eventdata, handles)
% hObject    handle to preamble_ms (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function duration_ms_Callback(hObject, eventdata, handles)
% hObject    handle to duration_ms (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of duration_ms as text
%        str2double(get(hObject,'String')) returns contents of duration_ms as a double
val = floor(str2double(get(hObject, 'String')));
val = max(val, 1);
set(hObject, 'String', val);

% --- Executes during object creation, after setting all properties.
function duration_ms_CreateFcn(hObject, eventdata, handles)
% hObject    handle to duration_ms (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes when uipanel4 is resized.
function uipanel4_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to uipanel4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function reward_duration_ms_Callback(hObject, eventdata, handles)
% hObject    handle to reward_duration_ms (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of reward_duration_ms as text
%        str2double(get(hObject,'String')) returns contents of reward_duration_ms as a double
val = floor(str2double(get(hObject, 'String')));
val = max(val, 1);
set(hObject, 'String', val);

% --- Executes during object creation, after setting all properties.
function reward_duration_ms_CreateFcn(hObject, eventdata, handles)
% hObject    handle to reward_duration_ms (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in choose_directory.
function choose_directory_Callback(hObject, eventdata, handles)
% hObject    handle to choose_directory (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
new_dir = uigetdir(pwd); % pwd returns present working directory
set(handles.WorkDir, 'String', new_dir);


% --- Executes when entered data in editable cell(s) in Difficulty_matrix.
function Difficulty_matrix_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to Difficulty_matrix (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.TABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)

table = get(hObject, 'Data');
row = eventdata.Indices(1);
col = eventdata.Indices(2);

% check for validity of most recently edited data
if isnan(eventdata.NewData)
    table(row, col) = eventdata.PreviousData;
else
    switch row
        case 1 % target angle was edited
            new_target = eventdata.NewData;
            new_target = max(new_target, 0);
            new_target = min(new_target, 180);
            table(row, col) = new_target;
        case 2 % target angle was edited
            new_window = eventdata.NewData;
            new_window = max(new_window, 1);
            new_window = min(new_window, 60);
            table(row, col) = new_window;
        case 3 % target angle was edited
            new_random_range_deg = eventdata.NewData;
            new_random_range_deg = max(new_random_range_deg, 0);
            new_random_range_deg = min(new_random_range_deg, 40);
            table(row, col) = new_random_range_deg;
        case 4 % hold duration_= (ms) was edited
            new_duration = eventdata.NewData;
            new_duration = max(new_duration, 10);
            new_duration = min(new_duration, 2000);
            table(row, col) = new_duration;
    end
end
set(hObject, 'Data', table);


% --- Executes on button press in test_opto_button.
function test_opto_button_Callback(hObject, eventdata, handles)
% hObject    handle to test_opto_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%clear serial read buffer
while (handles.port.BytesAvailable)
    fscanf(handles.port);
end
% engage opto for specified duration
cmd = sprintf("do %d", floor(str2double(get(handles.test_opto_ms, 'String'))));
fprintf(handles.port, cmd);
while (~handles.port.BytesAvailable)
end
while (handles.port.BytesAvailable)
    msg = fscanf(handles.port);
    fprintf(msg);
end

% --- Executes on button press in test_reward_button.
function test_reward_button_Callback(hObject, eventdata, handles)
% hObject    handle to test_reward_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of test_reward_button
%clear serial read buffer
while (handles.port.BytesAvailable)
    fscanf(handles.port);
end
% engage reward for specified duration
cmd = sprintf("dr %d", floor(str2double(get(handles.test_reward_ms, 'String'))));
fprintf(handles.port, cmd);
while (~handles.port.BytesAvailable)
end
while (handles.port.BytesAvailable)
    msg = fscanf(handles.port);
    fprintf(msg);
end


% --- Executes on button press in test_hardware_button.
function test_hardware_button_Callback(hObject, eventdata, handles)
% hObject    handle to test_hardware_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of test_hardware_button
if get(hObject, 'Value') == 1
    set(handles.hardware_test_panel, 'Visible', 1);
else
    set(handles.hardware_test_panel, 'Visible', 0);  
end



function test_opto_ms_Callback(hObject, eventdata, handles)
% hObject    handle to test_opto_ms (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of test_opto_ms as text
%        str2double(get(hObject,'String')) returns contents of test_opto_ms as a double
%clear serial read buffer
while (handles.port.BytesAvailable)
    fscanf(handles.port);
end
% engage reward for specified duration
cmd = sprintf("dr %d", floor(str2double(get(handles.test_reward_ms, 'String'))));
fprintf(handles.port, cmd);
while (~handles.port.BytesAvailable)
end
while (handles.port.BytesAvailable)
    msg = fscanf(handles.port);
    fprintf(msg);
end

% --- Executes during object creation, after setting all properties.
function test_opto_ms_CreateFcn(hObject, eventdata, handles)
% hObject    handle to test_opto_ms (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function test_reward_ms_Callback(hObject, eventdata, handles)
% hObject    handle to test_reward_ms (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of test_reward_ms as text
%        str2double(get(hObject,'String')) returns contents of test_reward_ms as a double


% --- Executes during object creation, after setting all properties.
function test_reward_ms_CreateFcn(hObject, eventdata, handles)
% hObject    handle to test_reward_ms (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%TODO test hardware test panel with programmed arduino
% --- Executes on button press in test_motor_button.
function test_motor_button_Callback(hObject, eventdata, handles)
% hObject    handle to test_motor_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

forward_boundary = nan;
reverse_boundary = nan;
center_angle = nan;

%clear serial read buffer
while (handles.port.BytesAvailable)
    fscanf(handles.port);
end
% move motor forward to obstacle and get position
fprintf(handles.port, 'dbf');
while (~handles.port.BytesAvailable)
end

msg = fscanf(handles.port);
fprintf(msg);
if contains(msg, "TURNED")
    forward_boundary = sscanf(msg, "TURNED TO OBSTACLE AT %f");
else
    set(handles.motor_test_outcome, 'String', "FAILURE");
    return;
end

% move motor reverse to obstacle and get position
fprintf(handles.port, 'dbf');
while (~handles.port.BytesAvailable)
end

msg = fscanf(handles.port);
fprintf(msg);
if contains(msg, "TURNED")
    reverse_boundary = sscanf(msg, "TURNED TO OBSTACLE AT %f");
else
    set(handles.motor_test_outcome, 'String', "FAILURE");
    return;
end

% move motor to center
fprintf(handles.port, 'da 90');
while (~handles.port.BytesAvailable)
end

msg = fscanf(handles.port);
fprintf(msg);
if contains(msg, "TURNED")
    center_angle = sscanf(msg, "TURNED TO ANGLE: %f");
else
    set(handles.motor_test_outcome, 'String', "FAILURE");
    return;
end

% check that the positions are as expected and update status accordingly
if (forward_boundary > 175) && (forward_boundary < 185)...
        && (reverse_boundary > -5) && (reverse_boundary < 5)...
        && (center_angle > 85) && (center_angle < 95)
    set(handles.motor_test_outcome, 'String', "SUCCESS");
else
    set(handles.motor_test_outcome, 'String', "FAILURE");
end


% --- Executes on button press in close_hardware_test_button.
function close_hardware_test_button_Callback(hObject, eventdata, handles)
% hObject    handle to close_hardware_test_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.hardware_test_panel, 'Visible', 0);  
