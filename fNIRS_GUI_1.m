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

% Last Modified by GUIDE v2.5 13-May-2014 23:32:50

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
    handles.plot = plot(t,y,'YDataSource','y','XDataSource','t');
    s1 = serial('COM29','BaudRate',9600,'DataBits',8,'Parity','None','FlowControl','None','StopBits',1);
    s1.BytesAvailableFcnCount = 40;
    s1.BytesAvailableFcnMode = 'byte';
    s1.BytesAvailableFcn = @(src, event) mycallback(src, event, hObject);
    fopen(s1);
    handles.serialport = s1;
    handles.inputdata = uint8([]);
    handles.output = hObject;
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

% --- Executes on button press in startButton.
function startButton_Callback(hObject, eventdata, handles)
    while true
       pause(.1); %give callback a chance to read some data
       handles = guidata(hObject); %The callback might have changed the values
       currentdata = handles.inputdata; %read pending data
       handles.inputdata = uint8([]); %clear pending data
       %now do something with the currentdata
       data = uint8(currentdata);
       data2=uint16(zeros(2,1));
       data2(1)=typecast(data(1:2),'uint16');
       data2(2)=typecast(data(3:4),'uint16');
       t = get(handles.plot,'XData');
       y = get(handles.plot,'YData');
       y = [y data2(1)];
       t = [t (t(end)+1)];
       set(handles.plot,'XData',t,'YData',y);
       refreshdata(handles.plot,'caller') 
       %drawnow
       % Update handles structure
       guidata(hObject, handles);
    end
end

function mycallback(src, event, FigureObject)
    handles = guidata(FigureObject);
    s1 = handles.serialport;
    thisdata = fread(s1, s1.BytesAvailable);
    handles.inputdata = [handles.inputdata; uint8(thisdata)];
    guidata(FigureObject, handles);
end