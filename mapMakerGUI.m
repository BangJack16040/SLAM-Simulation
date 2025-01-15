

function varargout = mapMakerGUI(varargin)

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @mapMakerGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @mapMakerGUI_OutputFcn, ...
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

function mapMakerGUI_OpeningFcn(hObject, eventdata, handles, varargin)

handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

global Lmks LmkGraphics Wpts WptGraphics DefaultText AxisDim RunTime ...
    Obstacles ObstaclesMidpoint World

World = [];

Lmks = [];

LmkGraphics = line(...
    'parent',handles.mainAxes, ...
    'linestyle','none', ...
    'marker','+', ...
    'color','b', ...
    'xdata',[], ...
    'ydata',[]);

Wpts = [];

WptGraphics = line(...
    'parent',handles.mainAxes, ...
    'marker','o', ...
    'color','r', ...
    'xdata',[], ...
    'ydata',[]);

Obstacles = [];
ObstaclesMidpoint = [];

DefaultText = 'Select a command...';

AxisDim = 10;
%set(handles.mainAxes, 'XLim', [-AxisDim,AxisDim], 'YLim', [-AxisDim,AxisDim]);
axes(handles.mainAxes)
axis([-AxisDim AxisDim -AxisDim AxisDim])
axis square

set(handles.AxisDimVar, 'String', AxisDim)

RunTime = 400;
set(handles.RunTimeVar, 'String', RunTime)

obsVertices = 3;
set(handles.obsVertices, 'String', obsVertices)

obsVelX = 0; obsVelY = 0;
set(handles.obsVelX, 'String', obsVelX)
set(handles.obsVelY, 'String', obsVelY)

% imagesc([], 'parent', handles.map1);
set(handles.map1, 'XTick', [], 'YTick', [])
% imagesc([], 'parent', handles.map2);
set(handles.map2, 'XTick', [], 'YTick', [])

% UIWAIT makes mapMakerGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = mapMakerGUI_OutputFcn(hObject, eventdata, handles) 


% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in AddLmk.
function AddLmk_Callback(hObject, eventdata, handles)

global Lmks AxisDim DefaultText
set(handles.helpBox, 'String', 'Click on the axes to place a landmark...')
clicked = 0;
while ~clicked
    [x,y]=ginputax(handles.mainAxes,1);
    if abs(x) < AxisDim && abs(y) < AxisDim
        Lmks = [Lmks [x;y]];
        axes(handles.mainAxes);
        %hold on
        plotItems(Lmks, 'landmarks');
        set(handles.helpBox, 'String', ...
            ['Landmark placed at:' char(10) ...
            '(' num2str(x) ', ' num2str(y) ')' char(10) DefaultText])
        clicked = 1;
    else
        set(handles.helpBox, 'String', ...
            'Landmark not placed! Click somewhere on the axes.')
    end
end

% --- Executes on button press in DelLmk.
function DelLmk_Callback(hObject, eventdata, handles)

global Lmks AxisDim DefaultText
set(handles.helpBox, 'String', 'Click a landmark to delete...')
clicked = 0;
while ~clicked
    if ~isempty(Lmks)
        p = ginputax(handles.mainAxes,1);
        if abs(p(1))<AxisDim && abs(p(2))<AxisDim
            i = nearestNeighbour(Lmks, p);
            % Remove nearest neighbour from Lmks
            lmk_deleted = Lmks(:,i);
            Lmks(:,i) = [];

            axes(handles.mainAxes);
            hold on
            plotItems(Lmks, 'landmarks');

            set(handles.helpBox, 'String', ...
                ['Landmark deleted at:' char(10) ...
                '(' num2str(lmk_deleted(1)) ', ' num2str(lmk_deleted(2)) ')' ...
                char(10) DefaultText])
            clicked = 1;
        else
            set(handles.helpBox, 'String', ...
                'No landmarks deleted! Click somewhere on the axes.')
        end
    else
        set(handles.helpBox, 'String', ...
            ['No landmarks to delete.' char(10) DefaultText])
        clicked = 1;
    end
end

% --- Executes on button press in doSLAM.
function doSLAM_Callback(hObject, eventdata, handles)

global Lmks Wpts DefaultText AxisDim Obstacles World Map1 Map2
if isempty(Lmks) || isempty(Wpts) || isempty(Obstacles)
    errordlg(['The map must consist of at least 1 landmark,' ...
        '1 waypoint and 1 obstacle'],'BOOM!')
else
    set(handles.helpBox, 'String', 'Executing SLAM...')
    World = SLAM(handles, AxisDim, Lmks, Wpts, Obstacles);
    set(handles.helpBox, 'String', ['SLAM simulation complete!' char(10) ...
        DefaultText])
    Map1 = World.gridmap_greyscale;
    Map2 = World.gridmap;
end

function plotItems(Items, ItemType)
global LmkGraphics WptGraphics
if strcmp(ItemType, 'landmarks')
    set(LmkGraphics, 'xdata', Items(1, :), 'ydata', Items(2, :))
elseif strcmp(ItemType, 'waypoints')
    set(WptGraphics, 'xdata', Items(1, :), 'ydata', Items(2, :))
end

function i = nearestNeighbour(Items, p)
diff2 = (Items(1,:)-p(1)).^2 + (Items(2,:)-p(2)).^2;
i= find(diff2 == min(diff2));
i= i(1);


% --- Executes on button press in LoadMap.
function LoadMap_Callback(hObject, eventdata, handles)

global Lmks Wpts Obstacles ObstaclesMidpoint DefaultText
clearMap_Callback(hObject, eventdata, handles)
seed = {'*.mat','MAT-files (*.mat)'};
[fn,pn] = uigetfile(seed, 'Load Map');
if fn==0, return, end

fnpn = strrep(fullfile(pn,fn), '''', '''''');
load(fnpn)
Lmks = lmks;
Wpts = wpts;
Obstacles = obs;
if ~isempty(Lmks)
    plotItems(Lmks, 'landmarks');
end
if ~ isempty(Wpts)
    plotItems(Wpts, 'waypoints');
end
if ~isempty(Obstacles)
    for i = 1:length(Obstacles)
        Obstacles(i).plot(handles.mainAxes);
        ObstaclesMidpoint(:, i) = [mean(Obstacles(i).vertices(1,:)); ...
            mean(Obstacles(i).vertices(2,:))];
    end
end

set(handles.helpBox, 'String', ['Map "' fn '" loaded!' char(10) ...
    DefaultText])



% --- Executes on button press in SaveMap.
function SaveMap_Callback(hObject, eventdata, handles)

global Lmks Wpts Obstacles DefaultText
lmks = Lmks;
wpts = Wpts;
obs = Obstacles;
if ~isempty(obs)
    for i = 1:length(obs)
        obs(i).graphics = []; % Reset graphics
    end
end
seed = {'*.mat','MAT-files (*.mat)'};
[fn,pn] = uiputfile(seed, 'Save Map');
if fn==0, return, end

fnpn = strrep(fullfile(pn,fn), '''', '''''');
save(fnpn, 'lmks', 'wpts', 'obs');
set(handles.helpBox, 'String', ['Map saved as "' fn '"!' char(10) ...
    DefaultText])

% --- Executes on button press in clearMap.
function clearMap_Callback(hObject, eventdata, handles)

global DefaultText
cla;
% Re-initialise map variables:
mapMakerGUI_OpeningFcn(hObject, eventdata, handles);
set(handles.helpBox, 'String', ['Map cleared!' char(10) DefaultText])

function AxisDimVar_Callback(hObject, eventdata, handles)

function AxisDimVar_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in setAxes.
function setAxes_Callback(hObject, eventdata, handles)

global AxisDim DefaultText
AxisDim = str2double(get(handles.AxisDimVar, 'String'));
axes(handles.mainAxes)
axis([-AxisDim AxisDim -AxisDim AxisDim])
set(handles.helpBox, 'String', ['Axis size set to:' char(10) ...
    num2str(AxisDim) char(10) DefaultText])


function RunTimeVar_Callback(hObject, eventdata, handles)

function RunTimeVar_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in setRunTime.
function setRunTime_Callback(hObject, eventdata, handles)

global RunTime DefaultText
RunTime = str2double(get(handles.RunTimeVar, 'String'));
set(handles.helpBox, 'String', ['Run time set to:' char(10) ...
    num2str(RunTime) char(10) DefaultText])


% --- Executes on button press in AddWpt.
function AddWpt_Callback(hObject, eventdata, handles)

global Wpts AxisDim DefaultText
set(handles.helpBox, 'String', 'Click on the axes to place a waypoint...')
clicked = 0;
while ~clicked
    [x,y]=ginputax(handles.mainAxes,1);
    if abs(x) < AxisDim && abs(y) < AxisDim
        Wpts = [Wpts [x;y]];
        axes(handles.mainAxes);
        %hold on
        plotItems(Wpts, 'waypoints');
        set(handles.helpBox, 'String', ...
            ['Waypoint placed at:' char(10) ...
            '(' num2str(x) ', ' num2str(y) ')' char(10) DefaultText])
        clicked = 1;
    else
        set(handles.helpBox, 'String', ...
            'Waypoint not placed! Click somewhere on the axes.')
    end
end

% --- Executes on button press in DelWpt.
function DelWpt_Callback(hObject, eventdata, handles)

global Wpts AxisDim DefaultText
set(handles.helpBox, 'String', 'Click a waypoint to delete...')
clicked = 0;
while ~clicked
    if ~isempty(Wpts)
        p = ginputax(handles.mainAxes,1);
        if abs(p(1))<AxisDim && abs(p(2))<AxisDim
            i = nearestNeighbour(Wpts, p);
            % Remove nearest neighbour from Lmks
            wpt_deleted = Wpts(:,i);
            Wpts(:,i) = [];
            
            axes(handles.mainAxes);
            hold on
            plotItems(Wpts, 'waypoints');

            set(handles.helpBox, 'String', ...
                ['Waypoint deleted at:' char(10) ...
                '(' num2str(wpt_deleted(1)) ', ' num2str(wpt_deleted(2)) ')' ...
                char(10) DefaultText])
            clicked = 1;
        else
            set(handles.helpBox, 'String', ...
                'No waypoints deleted! Click somewhere on the axes.')
        end
    else
        set(handles.helpBox, 'String', ...
            ['No waypoints to delete.' char(10) DefaultText])
        clicked = 1;
    end
end


% --- Executes on button press in AddObstacle.
function AddObstacle_Callback(hObject, eventdata, handles)

global Obstacles ObstaclesMidpoint DefaultText AxisDim

velX = str2double(get(handles.obsVelX, 'String'));
velY = str2double(get(handles.obsVelY, 'String'));

if isempty(Obstacles)
    Obstacles = Obstacle([], [velX; velY]);
else
    Obstacles(end+1) = Obstacle([], [velX; velY]);
end

vertices = str2double(get(handles.obsVertices, 'String'));
numObstacles = length(Obstacles);

set(handles.helpBox, 'String', ['Adding obstacle with' char(10) ...
    num2str(vertices) ' vertices.' char(10) DefaultText])

i = 1;
while i <= vertices
    p = ginputax(handles.mainAxes, 1);
    if abs(p(1))<AxisDim && abs(p(2))<AxisDim
        Obstacles(numObstacles).vertices(:, end+1) = p;
        Obstacles(numObstacles).plot(handles.mainAxes);
        i = i + 1;
    end
end

ObstaclesMidpoint(:, numObstacles) = [mean(Obstacles(numObstacles).vertices(1,:)); ...
    mean(Obstacles(numObstacles).vertices(2,:))];

set(handles.helpBox, 'String', ['Obstacle added!' char(10) DefaultText])

% --- Executes on button press in DelObstacle.
function DelObstacle_Callback(hObject, eventdata, handles)

global Obstacles ObstaclesMidpoint AxisDim DefaultText

set(handles.helpBox, 'String', 'Click an obstacle to delete...')
clicked = 0;
while ~clicked
    if ~isempty(Obstacles)
        p = ginputax(handles.mainAxes,1);
        if abs(p(1))<AxisDim && abs(p(2))<AxisDim
            i = nearestNeighbour(ObstaclesMidpoint, p);
            % Remove nearest obstacle
            Obstacles(i).vertices = [];
            Obstacles(i).plot(handles.mainAxes);
            Obstacles(i) = [];
            ObstaclesMidpoint(:, i) = [];

            axes(handles.mainAxes);

            set(handles.helpBox, 'String', ['Obstacle deleted!' char(10) ...
                DefaultText])
            
            clicked = 1;
        else
            set(handles.helpBox, 'String', ...
                'No obstacles deleted! Click somewhere on the axes.')
        end
    else
        set(handles.helpBox, 'String', ...
            ['No obstacles to delete.' char(10) DefaultText])
        clicked = 1;
    end
end


function obsVertices_Callback(hObject, eventdata, handles)

function obsVertices_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function obsVelX_Callback(hObject, eventdata, handles)

function obsVelX_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function obsVelY_Callback(hObject, eventdata, handles)

function obsVelY_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in saveGridMap.
function saveGridMap_Callback(hObject, eventdata, handles)

global DefaultText Map1 Map2 World

if ~isempty(World)
    mapType = get(handles.selectMap, 'Value');
    mapFormat = get(handles.saveGridMapType, 'Value');
    switch mapType
        case 1, % 8-bit map
            gridmap = Map1;
        case 2, % Tri-state mape
            gridmap = Map2;
    end
    
    switch mapFormat
        case 1, % Image
            gridmap = imresize(flipud(gridmap), round(World.map_res * 20), 'nearest');
            imwrite(gridmap, 'gridmap.tiff');
            % add screen size and comment
            set(handles.helpBox, 'String', ['Map saved as gridmap.tiff!' char(10) DefaultText])
        case 2, % .mat file
            map_res = World.map_res;
            seed = {'*.mat','MAT-files (*.mat)'};
            [fn,pn] = uiputfile(seed, 'Save Occupancy Grid Map');
            if fn==0, return, end

            fnpn = strrep(fullfile(pn,fn), '''', '''''');
            save(fnpn, 'gridmap', 'map_res');

            set(handles.helpBox, 'String', ['Grid map saved!' char(10) DefaultText])     
    end
else
    errordlg(['The map you are trying to save is currently empty.' ...
        'First run a simulation to generate a map.'],'Map not saved!')
end
        


% --- Executes on selection change in selectMap.
function selectMap_Callback(hObject, eventdata, handles)

function selectMap_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in saveGridMapType.
function saveGridMapType_Callback(hObject, eventdata, handles)

function saveGridMapType_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
