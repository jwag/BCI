function varargout = fNIRS_GUI_1(varargin)
% FNIRS_GUI_1 MATLAB code for fNIRS_GUI_1.fig
%      FNIRS_GUI_1, by itself, creates a new FNIRS_GUI_1 or raises the existing
%      singleton*.
%
%      H = FNIRS_GUI_1 returns the handle to a new FNIRS_GUI_1 or the handle to
%      the existing singleton*.
%
%      FNIRS_GUI_1('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in FNIRS_GUI_1.M with the given input arguments.
%
%      FNIRS_GUI_1('Property','Value',...) creates a new FNIRS_GUI_1 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before fNIRS_GUI_1_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to fNIRS_GUI_1_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help fNIRS_GUI_1

% Last Modified by GUIDE v2.5 14-May-2014 01:46:58

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @fNIRS_GUI_1_OpeningFcn, ...
                   'gui_OutputFcn',  @fNIRS_GUI_1_OutputFcn, ...
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
end

% --- Executes just before fNIRS_GUI_1 is made visible.
function fNIRS_GUI_1_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to fNIRS_GUI_1 (see VARARGIN)
    global y t
    t = 0;
    y = 0;
    handles.graph = false;
    handles.plot = plot(t,y,'YDataSource','y','XDataSource','t');
    handles.inputdata = uint8([]);
    handles.outputdata.t = t;
    handles.outputdata.y = y;
    handles.output = hObject;
    handles.counter = 0;
    handles.clear_input_flag = false;
    handles.clear_input_index = 1;
    s1 = serial('COM29','BaudRate',9600,'DataBits',8,'Parity','None','FlowControl','None','StopBits',1);
    s1.BytesAvailableFcnCount = 40;
    s1.BytesAvailableFcnMode = 'byte';
    s1.BytesAvailableFcn = @(src, event) mycallback(src, event, hObject, handles);
    handles.serialport = s1;
    fopen(s1);
    % Update handles structure
    guidata(hObject, handles);
% UIWAIT makes fNIRS_GUI_1 wait for user response (see UIRESUME)
% uiwait(handles.figure1);
end

% --- Outputs from this function are returned to the command line.
function varargout = fNIRS_GUI_1_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
    varargout{1} = handles.output;
end

% --- Executes on button press in Start.
function Start_Callback(hObject, eventdata, handles)
    handles.graph = true;
    status = get(handles.serialport,'Status');
    handles.inputdata = uint8([]); %clear pending data
    if~strcmp(status,'open')
        %handles.serialport.BytesAvailableFcn = @(src, event) mycallback(src, event, hObject,handles);
        fopen(handles.serialport);
    end
    guidata(hObject, handles);
    while handles.graph
       testdata = handles.inputdata;
       handles = guidata(hObject); %The callback might have changed the values
       currentdata = handles.inputdata(handles.clear_input_index:end);
       %currentdata = handles.inputdata; %read pending data
       t = handles.outputdata.t;
       y = handles.outputdata.y;
       if handles.clear_input_flag ~= true
           handles.clear_input_index = length(currentdata)+1;
           handles.clear_input_flag = true;
           % Update handles structure
           guidata(hObject, handles);
       end
       %now do something with the currentdata
       while ~isempty(currentdata)
           start_index = find_start(currentdata);
           if(start_index ~= -1 && length(currentdata)>=2+3)
               data = uint8(currentdata);
               data2=uint16(zeros(1,1));
               data2=typecast(data(start_index:start_index+1),'uint16');
               y = [y data2'];
               temp_time = linspace(t(end),t(end)+1,1);
               t = [t temp_time];
               currentdata = currentdata(start_index+2:end);
               if(handles.counter == 0)
                   set(handles.plot,'XData',t,'YData',y);
                   refreshdata(handles.plot,'caller');
                   drawnow;
                   handles.counter = 0;
               else
                   handles.counter = handles.counter+1;
               end
           end
       end
       % Get a new copy of the latest data and publish our changes to the
       % handles
       new_handles = guidata(hObject);
       % we are only using these but we need to get the latest
       % handles.graph
       new_handles.outputdata.t = t;
       new_handles.outputdata.y = y;
       new_handles.counter = handles.counter;
       %new_handles.plot = handles.plot;
       % handles.plot????
       % Update handles structure
       guidata(hObject, new_handles);
    end
end

function mycallback(src, event, FigureObject,handles)
    handles = guidata(FigureObject);
    s1 = handles.serialport;
    if(handles.clear_input_flag == true)
        handles.inputdata = handles.inputdata(handles.clear_input_index:end);
        handles.clear_input_index = 1;
        handles.clear_input_flag = 0;
        guidata(FigureObject, handles);
    end
    if s1.BytesAVailable~=0
        thisdata = fread(s1, s1.BytesAvailable);
        handles.inputdata = [handles.inputdata; uint8(thisdata)];
        guidata(FigureObject, handles);
    end
end


% --- Executes on button press in Stop.
function Stop_Callback(hObject, eventdata, handles)
% hObject    handle to Stop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    handles.graph = false;
    guidata(hObject, handles);
    status = get(handles.serialport,'Status');
    if strcmp(status,'open')
        fclose(handles.serialport);
        %handles.serialport.BytesAvailableFcn = @null;
    end
    guidata(hObject, handles);
    %handles.output = get(handles.plot,'XData');
    %handles.output = get(handles.plot,'YData');
end

function [start_index] = find_start(data)
    start_index = -1;
    index = 1;
    i = 0;
    len = length(data);
    while(index <= len && start_index == -1)
        switch i
            case 0
                if(char(data(index)) == '<')
                    i = 1;
                else
                    i=0;
                end
                index = index+1;
            case 1
                if(char(data(index)) == '(')
                    i = 2;
                else
                    i = 0;
                end
                index = index+1;
            case 2
                if(char(data(index)) == '{')
                    start_index = index+1;
                else
                    i = 0;
                    index = index+1;
                end
        end
    end
end