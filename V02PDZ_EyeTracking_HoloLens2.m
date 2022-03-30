%% Version: 06.03.2020 - PDZ-HoloLens EyeTracking Script
%% Questions: marwacht@ethz.ch
% Script return video with additional GazePoint
%%%%%%
% Check First:
% Use Matlab 2019.b (2019.a does not work)
% Use Unity 2019.2.19 for EyeGaze Project
% Visual Studio 2019
% If using new Hololens 2, Do a Camera Calibration


%%%%%%
% How to use PDZ-EyeTracking_HoloLens2.m:
% 1. Record Video and Data with HoloLens 2
% 2. Download it from Windows Device Portal (Call HoloLens IP in Browser (e.g.
%    192.168.0.12) -> Local App Data -> EyeTracking -> Local State -> Order
%    Files by "Date Created"
% 3. Files must be saved in Current Matlab-Folder
% 4. Press Run (in Matlab)
%%%%%%

%dwelltimesSum: shows summed up dwell times for each AOI
%dweltimesSeperated: shows dwell time for each AOI (chronological order)
%EyeGazeTargets: List with GazeTarget for each frame (corresponds to time
    %stamp ofEyeGazeDataMatrix
%EyeGazeDataMatrix: Contains Direction, HL-Position, HL-Rotation

% EyeGazeDataMatrix - Matrix Rows: 
%   1  0
%   2  Time.time
%   3  GazeDirection.x
%   4  GazeDirection.y
%   5  GazeDirection.z
%   6  GazeOrigin.x
%   7  GazeOrigin.y
%   8  GazeOrigin.z
%   9  Camera.main.transform.rotation.x
%   10  Camera.main.transform.rotation.y
%   11 Camera.main.transform.rotation.z,
%   12 Camera.main.transform.rotation.w,

% EyeGazeTargets Cell Rows:
%   1 IsEyeGazeValid:  Valid: HoloLens can detect gaze position
%                      NotValid: Error, gaze not correct (eyes closed?)
%   2 GazeTarget (Name of hit GameObject) (Background: nothing hit)

%%import Data
clear;
clc;
close all;

f = fopen ('211210_22-05-48_GazeData.txt');
C = textscan(f, '%s', 'Delimiter', ',');

fclose(f);
EyeGazeDataCell = (reshape(C{1,1},11,[size(C{1,1},1)/11]))';

Videofile = uigetfile('*.mp4');
%EyeGazeDataFile = uigetfile('*.txt');
%EyeGazeDataVector = importdata(EyeGazeDataFile, ',');
%EyeGazeDataCell = (reshape(EyeGazeDataVector.textdata, [14, size(EyeGazeDataVector.textdata,2)/14]))';
EyeGazeDataMatrix = str2double(EyeGazeDataCell(2:end,1:11));
%EyeGazeTargets = EyeGazeDataCell(:,12)
EyeGazeDataMatrix(:,1)=EyeGazeDataMatrix(:,1) - EyeGazeDataMatrix(1,1);

prompt = {'Enter filename for Eye-Gaze-Video'};
dlgtitle = 'Filename Input';
definput = {'EyeGaze-complete-1'};
opts.Interpreter = 'none';
dlgNameInput = string(inputdlg(prompt,dlgtitle,[1 30],definput,opts));
videoFileName = strcat(dlgNameInput,'.mp4');
disp('Step 1: Data Imported');

%% Initialize CameraIntrinsics and preallocate space
f2 = waitbar(0.1, "Calculating GazePoint - Please wait... ");
movegui(f2,[300 800]);
v = VideoReader(Videofile);
if v.width == 1272   %1272 %High Resolution Video (Expensive)
    VideoResolution = 1;
    %HoloLens Camera Intrinsics:
    K =  [  1712    0       1130;
            0       1718    589;
            0       0       1   ];
elseif v.width == 896 %Low (DVD) Resolution - standard resolution, HoloLens12
    VideoResolution = 2;
    K = [   655.73  0       428.67;
            0       674.10  220.75;
            0     	0       1   ];
else
    error("VideoResolution not correct");
end

%Videorecording and GazeData do not start synchornously, therefore Offset is needed   

VideoOffset = EyeGazeDataMatrix(end,1)-v.Duration
%VideoOffset = 1000;
[~,idx]= min(abs(EyeGazeDataMatrix(:,1)-VideoOffset))

% Preallocate space
nFrames = ceil(v.FrameRate*v.Duration);
s(nFrames) = struct('cdata',[],'colormap',[]);
WorldCoordinateMatrix = zeros(1,3);

% Set up figure, axes to hold image and Eye-Gaze plot
hFig = figure('MenuBar','none','Units','pixels','Position',[100 100 v.Width v.Height]);
hAx = axes('Parent',hFig,'Units','pixels','Position',[00 00 v.Width v.Height],...
    'NextPlot','add','Visible','off','XTick',[],'YTick',[]);
hIm = image(uint8(zeros(v.Height,v.Width,3)),'Parent',hAx);
hLine(1) = plot(hAx,1,1,'o','MarkerSize',10,'MarkerEdgeColor','g', 'LineWidth', 3);

% Loop through video, grabbing frames and updating plots
k = 1;              % video frame number
r = idx;            % row of EyeGazeDataMatrix
q = v.NumFrames;    % total Number of Videoframes


counter = 0;

%% plot GazePoint on Frames
U = [];
V = [];
while hasFrame(v)
    counter = counter + 1
    %Check if Timestamp of Video matches timestamp of EyeGazeDataMatrix,
    %otherwise change row of EyeGazeDataMatrix
    while(v.CurrentTime-0.1 > EyeGazeDataMatrix(r,1)-VideoOffset)
        r = r + 1;
    end
    while(v.CurrentTime+0.1 < EyeGazeDataMatrix(r,1)-VideoOffset)
        r = r - 1;
    end
    %Position data delayed by 12 data points. Check row 
    if r+12 > size(EyeGazeDataMatrix, 1)
        r = size(EyeGazeDataMatrix, 1)-12;
    end
    ViewDistance = 2.0; %Desired Distance in meters
    
    %Quaternion of HoloLens Rotation (w,x,y,z) --> 3x3 Rotation Matrix
    R_CamToWorld = (quat2rotm([EyeGazeDataMatrix(r,11) EyeGazeDataMatrix(r,8:10)]))';

    
    %negative Translation (HoloLens Position in World Coordinates (x,y,z),
    %Translation from Eye to webcam added
    T_CamToWorld = R_CamToWorld * (-EyeGazeDataMatrix(r+12, 5:7)' + [0.005, 0.1, 0]') ;
    
    %World Coordinates of Gaze Point. Position of HoloLens-Webcam + 
    %  VectorGazedirection)*desired view distance
    WorldCoordinates = [ EyeGazeDataMatrix(r,5:7)' + EyeGazeDataMatrix(r,2:4)'*ViewDistance;1];

    %Projection Matrix
    M = K*[R_CamToWorld T_CamToWorld];
   
    PixelCoordinates = M*WorldCoordinates; %lambda * (u,v,1)'
    PixelCoordinates = PixelCoordinates ./PixelCoordinates(3); % = (u,v,1)
    

    
    if (PixelCoordinates(1) < 0 || PixelCoordinates(1) > v.Width || PixelCoordinates(2) < 0 ...
            || PixelCoordinates(2) > v.Height)
        PixelCoordinates(1) = 0;
        PixelCoordinates(2) = 0;
    end
    
    im = readFrame(v); %call next frame
    hIm.CData = flip(im); %flip image (upside down)
    hLine(1).XData = PixelCoordinates(1); % = u
    hLine(1).YData = PixelCoordinates(2)-23; % = v

    U = [U, PixelCoordinates(1)];
    V = [V, PixelCoordinates(2)-23];
    
    
   
    

    %txt = sprintf('%.2f ; AOI: %s', [v.CurrentTime, string(EyeGazeTargets(r,1))]);
%    videoText = text(20, 20, txt , 'FontSize', 10, 'Color', [1 1 1] );
    drawnow
    
    
    % Save the frame in structure for later saving to video file    
    s(k) = getframe;
    k = k+1;  
    r = r+1;
    waitbar(k/q*0.9, f2);
   % delete(videoText);
end

filename = 'coordinates.xlsx';

for i = 1:length(U)
   if i == 1
       Table = {'X', 'Y'; U(i) V(i);};
       writecell(Table, filename);
   else
       Table = {U(i) V(i);};
       writecell(Table, filename, 'WriteMode', 'append');

   end

end

disp('Step 2: frames temporarly stored')
waitbar(k/q*0.9,f2, "Saving Video...");
close(figure(1));
% Remove any unused structure array elements
s(k:end) = [];

%% Write to a video file and save it
vOut = VideoWriter(videoFileName{1,1},'MPEG-4');
vOut.FrameRate = v.FrameRate;
open(vOut)
for k = 1:numel(s)
    writeVideo(vOut,s(k))
    waitbar(k/numel(s)*0.1+0.9)
end

waitbar(1.0,f2, "Finishing..");
close(vOut)
close(f2)
disp('Step 3: video saved')
f = msgbox({'Success!';'Eye-Tracking Video Complete. Saved as:'; videoFileName{1,1};'Success'});
movegui(f,[300 800]);

%% CalculateDwellTime

gazeDurationStart = 0;
gazeDurationStop = 0;
dwelltime = 0;
%preallocating Space:
dwelltimeSeperated = cell(100, 4); %shows cronological AOI and seperated dwell time
dwelltimeSeperated(1,:) = [{'AOI'},{'gazeStartTime'},{'gazeStopTime'},{'dwelltime'}];
dwelltimesSum = cell(10, 2); %summed up dwell time for each AOI
dwelltimesSum(1,:) = [{'AOI'},{'Total-DwellTime'}];
IndexDwelltimeSeperated = 2; 
IndexDwelltimesSum = 2;

%for i = 2:(size(EyeGazeDataMatrix, 1)-1)
%    if ismember(EyeGazeTargets(i-1,1), EyeGazeTargets(i,1)) == false
%        gazeDurationStart = EyeGazeDataMatrix(i,1);
%    end
%    if ismember(EyeGazeTargets(i,1), EyeGazeTargets(i+1,1)) == false
%        gazeDurationStop = EyeGazeDataMatrix(i,1);
%        dwelltime = gazeDurationStop - gazeDurationStart;       
%        dwelltimeSeperated(IndexDwelltimeSeperated, :) = [EyeGazeTargets(i,1) ...
%            gazeDurationStart gazeDurationStop dwelltime];
%        IndexDwelltimeSeperated = IndexDwelltimeSeperated + 1;        
%        AOIIndex = find(strcmp(dwelltimesSum, EyeGazeTargets(i,1)));
%        if AOIIndex > 1
%            dwelltimesSum{AOIIndex,1} = dwelltimesSum{AOIIndex,1} + dwelltime;         
%        else
%            dwelltimesSum(IndexDwelltimesSum, :) = [EyeGazeTargets(i,1) dwelltime];
%            IndexDwelltimesSum = IndexDwelltimesSum + 1;
%        end
%    end
%       
%        
%end

%% Save DwellTimes in XLS-File
xlsFileName = strcat(dlgNameInput,'.xls');
warning( 'off', 'MATLAB:xlswrite:AddSheet');
xlswrite( xlsFileName{1,1}, dwelltimeSeperated, 'DwelltimeSeperated')
xlswrite( xlsFileName{1,1}, dwelltimesSum, 'DwelltimesSum')
disp('Step 4: XLS-File Generated')