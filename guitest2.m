
function varargout = guitest2(varargin)
% GUITEST2 MATLAB code for guitest2.fig
%      GUITEST2, by itself, creates a new GUITEST2 or raises the existing
%      singleton*.
%
%      H = GUITEST2 returns the handle to a new GUITEST2 or the handle to
%      the existing singleton*.
%
%      GUITEST2('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUITEST2.M with the given input arguments.
%
%      GUITEST2('Property','Value',...) creates a new GUITEST2 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before guitest2_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to guitest2_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help guitest2

% Last Modified by GUIDE v2.5 30-Nov-2018 18:26:31

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @guitest2_OpeningFcn, ...
                   'gui_OutputFcn',  @guitest2_OutputFcn, ...
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


% --- Executes just before guitest2 is made visible.
function guitest2_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to guitest2 (see VARARGIN)

% Choose default command line output for guitest2
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes guitest2 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = guitest2_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function x_Callback(hObject, eventdata, handles)
% hObject    handle to x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of x as text
%        str2double(get(hObject,'String')) returns contents of x as a double


% --- Executes during object creation, after setting all properties.
function x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
l(1) = Link([pi, 0.2755, 0, pi/2, 0]);
l(2) = Link([-pi/2, 0, 0.41, pi, 0]);
l(3) = Link([-pi/2, -0.0098, 0, pi/2, 0]);
l(4) = Link([0, -0.2841, 0, pi/2, 0]);
l(5) = Link([0, 0, 0, pi/2, 0]);
l(6) = Link([pi, 0.2341, 0, pi/2, 0]);
R = SerialLink(l);
x1=str2double(get(handles.x,'string'));
y1=str2double(get(handles.y,'string'));
z1=str2double(get(handles.z,'string'));
r1=str2double(get(handles.r,'string'));
p1=str2double(get(handles.p,'string'));
w1=str2double(get(handles.w,'string'));
q=pos(x1,y1,z1,r1,p1,w1,R);
R.plot(q);
function ik =pos(x,y,z,r,p,w,R)
t1=r*(pi/180);
t2=p*(pi/180);
t3=w*(pi/180);
rz=rotx(t3);
ry=roty(t2);
rx=rotz(t1);
rzyx=rz*ry*rx;
o = [x;y;z];
T= [rzyx  o ; 0 0 0 1];
ik=R.ikunc(T);




function y_Callback(hObject, eventdata, handles)
% hObject    handle to y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of y as text
%        str2double(get(hObject,'String')) returns contents of y as a double


% --- Executes during object creation, after setting all properties.
function y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function z_Callback(hObject, eventdata, handles)
% hObject    handle to z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of z as text
%        str2double(get(hObject,'String')) returns contents of z as a double


% --- Executes during object creation, after setting all properties.
function z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function r_Callback(hObject, eventdata, handles)
% hObject    handle to r (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of r as text
%        str2double(get(hObject,'String')) returns contents of r as a double


% --- Executes during object creation, after setting all properties.
function r_CreateFcn(hObject, eventdata, handles)
% hObject    handle to r (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function p_Callback(hObject, eventdata, handles)
% hObject    handle to p (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of p as text
%        str2double(get(hObject,'String')) returns contents of p as a double


% --- Executes during object creation, after setting all properties.
function p_CreateFcn(hObject, eventdata, handles)
% hObject    handle to p (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function w_Callback(hObject, eventdata, handles)
% hObject    handle to w (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of w as text
%        str2double(get(hObject,'String')) returns contents of w as a double


% --- Executes during object creation, after setting all properties.
function w_CreateFcn(hObject, eventdata, handles)
% hObject    handle to w (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q1_Callback(hObject, eventdata, handles)
% hObject    handle to q1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q1 as text
%        str2double(get(hObject,'String')) returns contents of q1 as a double


% --- Executes during object creation, after setting all properties.
function q1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q2_Callback(hObject, eventdata, handles)
% hObject    handle to q2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q2 as text
%        str2double(get(hObject,'String')) returns contents of q2 as a double


% --- Executes during object creation, after setting all properties.
function q2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q3_Callback(hObject, eventdata, handles)
% hObject    handle to q3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q3 as text
%        str2double(get(hObject,'String')) returns contents of q3 as a double


% --- Executes during object creation, after setting all properties.
function q3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q4_Callback(hObject, eventdata, handles)
% hObject    handle to q4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q4 as text
%        str2double(get(hObject,'String')) returns contents of q4 as a double


% --- Executes during object creation, after setting all properties.
function q4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q5_Callback(hObject, eventdata, handles)
% hObject    handle to q5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q5 as text
%        str2double(get(hObject,'String')) returns contents of q5 as a double


% --- Executes during object creation, after setting all properties.
function q5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q6_Callback(hObject, eventdata, handles)
% hObject    handle to q6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q6 as text
%        str2double(get(hObject,'String')) returns contents of q6 as a double


% --- Executes during object creation, after setting all properties.
function q6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
l(1) = Link([pi, 0.2755, 0, pi/2, 0]);
l(2) = Link([-pi/2, 0, 0.41, pi, 0]);
l(3) = Link([-pi/2, -0.0098, 0, pi/2, 0]);
l(4) = Link([0, -0.2841, 0, pi/2, 0]);
l(5) = Link([0, 0, 0, pi/2, 0]);
l(6) = Link([pi, 0.2341, 0, pi/2, 0]);
R = SerialLink(l);
q1=str2double(get(handles.q1,'string'));
q2=str2double(get(handles.q2,'string'));
q3=str2double(get(handles.q3,'string'));
q4=str2double(get(handles.q4,'string'));
q5=str2double(get(handles.q5,'string'));
q6=str2double(get(handles.q6,'string'));
q=[q1,q2,q3,q4,q5,q6]*(pi/180)
R.plot(q);



function r1_Callback(hObject, eventdata, handles)
% hObject    handle to r1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of r1 as text
%        str2double(get(hObject,'String')) returns contents of r1 as a double


% --- Executes during object creation, after setting all properties.
function r1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to r1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
l(1) = Link([pi, 0.2755, 0, pi/2, 0]);
l(2) = Link([-pi/2, 0, 0.41, pi, 0]);
l(3) = Link([-pi/2, -0.0098, 0, pi/2, 0]);
l(4) = Link([0, -0.2841, 0, pi/2, 0]);
l(5) = Link([0, 0, 0, pi/2, 0]);
l(6) = Link([pi, 0.2341, 0, pi/2, 0]);
R = SerialLink(l);
r1=str2double(get(handles.r1,'string'));
if r1>0.8
for i=0:0.3:2*pi; 
  xp=r1*cos(i)
  yp=r1*sin(i)
  q=pos1(xp,yp,0,0,90,0,R)
  T=fkine(R,q(1,1:6))
  plot3(T.t(1),T.t(2),T.t(3),'*r')
  hold on
  R.plot(q)   
end
else
for i=0:0.3:2*pi; 
  xp=r1*cos(i)
  yp=r1*sin(i)
  q=pos1(xp,yp,0.5,180,0,90,R)
  T=fkine(R,q(1,1:6))
  plot3(T.t(1),T.t(2),T.t(3),'*r')
  hold on
  R.plot(q)
end
end
R.plot([0,0,0,0,0,0])
function ik =pos1(x,y,z,r,p,w,R)
t1=r*(pi/180);
t2=p*(pi/180);
t3=w*(pi/180);
rz=rotx(t3);
ry=roty(t2);
rx=rotz(t1);
rzyx=rz*ry*rx;
o = [x;y;z];
T= [rzyx  o ; 0 0 0 1];
ik=R.ikunc(T);



function len_Callback(hObject, eventdata, handles)
% hObject    handle to len (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of len as text
%        str2double(get(hObject,'String')) returns contents of len as a double


% --- Executes during object creation, after setting all properties.
function len_CreateFcn(hObject, eventdata, handles)
% hObject    handle to len (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
l(1) = Link([pi, 0.2755, 0, pi/2, 0]);
l(2) = Link([-pi/2, 0, 0.41, pi, 0]);
l(3) = Link([-pi/2, -0.0098, 0, pi/2, 0]);
l(4) = Link([0, -0.2841, 0, pi/2, 0]);
l(5) = Link([0, 0, 0, pi/2, 0]);
l(6) = Link([pi, 0.2341, 0, pi/2, 0]);
R = SerialLink(l);
  sl=str2double(get(handles.len,'string'));
  x1=sl/2
  x2=-sl/2
  y1=sl/2;
  y2=-sl/2;
  n=round((y1-y2)/0.1+(1))
  xp=x1:-0.1:x2
  yp=y1:-0.1:y2
for i=1:1:n 
  q=pos2(xp(i),y1,0.8,-180,0,-90,R);
  T=fkine(R,q(1,1:6));
  plot3(T.t(1),T.t(2),T.t(3),'r*');
  hold on;
  R.plot(q);
end
for i=1:1:n
 
  q=pos2(xp(n),yp(i),0.8,-180,0,-90,R);
  T=fkine(R,q(1,1:6));
  plot3(T.t(1),T.t(2),T.t(3),'r*');
  hold on;
  R.plot(q);
end
for i=1:1:n
  q=pos2(xp(n+1-i),yp(n),0.8,-180,0,-90,R);
  T=fkine(R,q(1,1:6));
  plot3(T.t(1),T.t(2),T.t(3),'r*');
  hold on;
  R.plot(q);
end
for i=1:1:n 
  q=pos2(xp(1),yp(n+1-i),0.8,-180,0,-90,R);
  T=fkine(R,q(1,1:6));
  plot3(T.t(1),T.t(2),T.t(3),'r*');
  hold on;
  R.plot(q);
end
function ik =pos2(x,y,z,r,p,w,R)
t1=r*(pi/180);
t2=p*(pi/180);
t3=w*(pi/180);
rz=rotx(t3);
ry=roty(t2);
rx=rotz(t1);
rzyx=rz*ry*rx;
o = [x;y;z];
T= [rzyx  o ; 0 0 0 1];
ik=R.ikunc(T);


% --- Executes on button press in cc.
function cc_Callback(hObject, eventdata, handles)
% hObject    handle to cc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
l(1) = Link([pi, 0.2755, 0, pi/2, 0]);
l(2) = Link([-pi/2, 0, 0.41, pi, 0]);
l(3) = Link([-pi/2, -0.0098, 0, pi/2, 0]);
l(4) = Link([0, -0.2841, 0, pi/2, 0]);
l(5) = Link([0, 0, 0, pi/2, 0]);
l(6) = Link([pi, 0.2341, 0, pi/2, 0]);
R = SerialLink(l);
r2=str2double(get(handles.r1,'string'));
for rl=r2:-0.05:0.05
for i=0:0.1:2*pi; 
  xp=rl*cos(i)
  yp=rl*sin(i)
  q=pos3(xp,yp,0.5,180,0,90,R)
  T=fkine(R,q(1,1:6))
  plot3(T.t(1),T.t(2),T.t(3),'*r')
  hold on
  R.plot(q)
end
end
function ik =pos3(x,y,z,r,p,w,R)
t1=r*(pi/180);
t2=p*(pi/180);
t3=w*(pi/180);
rz=rotx(t3);
ry=roty(t2);
rx=rotz(t1);
rzyx=rz*ry*rx;
o = [x;y;z];
T= [rzyx  o ; 0 0 0 1];
ik=R.ikunc(T);


% --- Executes on button press in ccs.
function ccs_Callback(hObject, eventdata, handles)
% hObject    handle to ccs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
l(1) = Link([pi, 0.2755, 0, pi/2, 0]);
l(2) = Link([-pi/2, 0, 0.41, pi, 0]);
l(3) = Link([-pi/2, -0.0098, 0, pi/2, 0]);
l(4) = Link([0, -0.2841, 0, pi/2, 0]);
l(5) = Link([0, 0, 0, pi/2, 0]);
l(6) = Link([pi, 0.2341, 0, pi/2, 0]);
R = SerialLink(l);
s=str2double(get(handles.len,'string'));
sl=s*10;
for sl=sl:-1:0
  x1=sl/2;
  x2=-sl/2;
  y1=sl/2;
  y2=-sl/2; 
  n=round((y1-y2)/1+(1))
  xp1=x1:-1:x2;
  xp = xp1/10
  yp1=y1:-1:y2;
  yp = yp1/10
for i=1:1:n 
  q=pos(xp(i),yp(1),0.8,-180,0,-90,R);
  T=fkine(R,q(1,1:6));
  plot3(T.t(1),T.t(2),T.t(3),'r*');
  hold on;
  R.plot(q);
end
for i=1:1:n
  q=pos(xp(n),yp(i),0.8,-180,0,-90,R);
  T=fkine(R,q(1,1:6));
  plot3(T.t(1),T.t(2),T.t(3),'r*');
  hold on;
  R.plot(q);
end
for i=1:1:n
  q=pos4(xp(n+1-i),yp(n),0.8,-180,0,-90,R);
  T=fkine(R,q(1,1:6));
  plot3(T.t(1),T.t(2),T.t(3),'r*');
  hold on;
  R.plot(q);
end
for i=1:1:n 
  q=pos(xp(1),yp(n+1-i),0.8,-180,0,-90,R);
  T=fkine(R,q(1,1:6));
  plot3(T.t(1),T.t(2),T.t(3),'r*');
  hold on;
  R.plot(q);
end
end
R.plot([0,0,0,0,0,0]);
function ik =pos4(x,y,z,r,p,w,R)
t1=r*(pi/180);
t2=p*(pi/180);
t3=w*(pi/180);
rz=rotx(t3);
ry=roty(t2);
rx=rotz(t1);
rzyx=rz*ry*rx;
o = [x;y;z];
T= [rzyx  o ; 0 0 0 1];
ik=R.ikunc(T);