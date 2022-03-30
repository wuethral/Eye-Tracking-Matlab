%% Back-Projections

% Data Recording - Matrix Rows: 
%   1  ImageNumber, starting at 0
%   2  ImageName
%   3  CameraPosition.x (before)
%   4  CameraPosition.y (before)
%   5  CameraPosition.z (before)
%   6  CameraRotation.x (before)
%   7  CameraRotation.y (before)
%   8  CameraRotation.z (before)
%   9  CameraRotation.w (before)
%   10 CameraPosition.x (after)
%   11 CameraPosition.y (after)
%   12 CameraPosition.z (after)
%   13 CameraRotation.x (after)
%   14 CameraRotation.y (after)
%   15 CameraRotation.z (after)
%   16 CameraRotation.w (after)
%   17 Point_1_Position.x
%   18 Point_1_Position.y
%   19 Point_1_Position.z
%   20 Point_2_Position.x
%   21 Point_2_Position.y
%   22 Point_2_Position.z
%   23 Point_3_Position.x
%   24 Point_3_Position.y
%   25 Point_3_Position.z
%   26 Point_4_Position.x
%   27 Point_4_Position.y
%   28 Point_4_Position.z


%%import Data
clear;
close all;

f = fopen ('DataRecording20200908_01-18-04.txt');
C = textscan(f, '%s', 'Delimiter', ',');
fclose(f);

% number of input columns
inputCols = 28;

%EyeGazeDataCell = (reshape(C{1,1},13,[size(C{1,1},1)/13]))';
DataCell = (reshape(C{1,1},inputCols,[size(C{1,1},1)/inputCols]))';
size(DataCell)

DataMatrix = str2double(DataCell(:,1:inputCols));
size(DataMatrix)


%% Initialize CameraIntrinsics and preallocate space


%HoloLens Camera Intrinsics:
K =  [ 2931    0       1908;
       0       2930    1037;
       0       0       1   ];

%% plot 
row =1; %image id in folder
num_points = 4;
PixelCoordinateMatrix = [2,num_points];

%first column number containing camera position/rotation
%before photo capture = 3; after photo capture = 10;
camera_time = 3;


%Quaternion of HoloLens Rotation (w,x,y,z) --> 3x3 Rotation Matrix
R_CamToWorld = (quat2rotm([DataMatrix(row,camera_time+6) DataMatrix(row, (camera_time+3:camera_time+5))]))';

%negative Translation (HoloLens Position in World Coordinates (x,y,z),
T_CamToWorld = R_CamToWorld * (-DataMatrix(row, camera_time:(camera_time+2)))';

for i = 1:num_points
    
    %column number of first point
    x_col_point_1 = 17+(i-1)*3;
    
    %World Coordinates of Gaze Point P(i)
    WorldCoordinates = [DataMatrix(row, x_col_point_1:(x_col_point_1+2))';1];

    %Projection Matrix
    M = K*[R_CamToWorld T_CamToWorld];

    disp("M: "+M)
    disp("WorldCoordinates: "+WorldCoordinates)

    PixelCoordinates = M*WorldCoordinates; %lambda * (u,v,1)'
    disp(PixelCoordinates)
    PixelCoordinates = PixelCoordinates ./PixelCoordinates(3); % = (u,v,1)
    disp(PixelCoordinates)
    
    PixelCoordinateMatrix(1, i) = PixelCoordinates(1);
    PixelCoordinateMatrix(2, i) = PixelCoordinates(2);
    
end

%% Get images

img_index = row-1;
imagePath = "CapturedImage" + img_index + ".jpg";
img = imread(imagePath);
img_Width = size(img,1);
img_Height = size(img,2);

for i = 1:num_points

    PixelCoordinates(1) = PixelCoordinateMatrix(1, i);
    PixelCoordinates(2) = PixelCoordinateMatrix(2, i);
    
    if (PixelCoordinates(1) < 0 || PixelCoordinates(1) > img_Width || PixelCoordinates(2) < 0 ...
            || PixelCoordinates(2) > img_Height)
        PixelCoordinates(1) = 0;
        PixelCoordinates(2) = 0;
    end
end 

img=flip(img);

imshow(img);  
r = 20;
hold on;
theta = 0 : (2 * pi / 10000) : (2 * pi);
for i = 1:num_points
    
    PixelCoordinates(1) = PixelCoordinateMatrix(1, i);
    PixelCoordinates(2) = PixelCoordinateMatrix(2, i);
    
    pline_x = r * cos(theta) + PixelCoordinates(1);
    pline_y = r * sin(theta) + PixelCoordinates(2);
    k = ishold;
    plot(pline_x, pline_y, '*');
end
    
%hold off;
    
    