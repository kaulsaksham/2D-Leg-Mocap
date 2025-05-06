%% Undistort GoPro Footage
% This program undistorts a video given the GoPro Hero 9's distortion
% parameters.
%
% NOTE: In order for this program to work, Computer Vision Toolbox and 
% intrinsic distortion parameters are necessary. To find these, use the 
% MATLAB Camera Calibrator app with a calibration checkerboard.
% 
% Also: this will only work with the GoPro due to its unique video
% resolution.

clc;
clear;
close all;

% Enter file name of video you would like to undistort
videoReader = VideoReader('Subject4_1.mp4'); % here

% Enter intrinsic distortion parameters
parameters = subject3_4_Params; % here

% load Video Player tool
videoPlayer = vision.VideoPlayer('Position', [0,0,500,500]);

% Enter file path where you would like undistorted video to be saved
videoFWriter = vision.VideoFileWriter(...
    'C:\Users\saksh\Downloads\MRRL\Subject1Recordings\UndistortedSubject4_1.mp4', ... % here
    'FrameRate', videoReader.FrameRate, 'FileFormat', 'MPEG4');

% begin undistortion
while hasFrame(videoReader)
    frame = readFrame(videoReader);
    croppedFrame = imcrop(frame, [0, 5, 2704, 2027]);
    undistortedObjectFrame = undistortImage(croppedFrame, parameters);
    videoPlayer(undistortedObjectFrame);
    videoFWriter(undistortedObjectFrame);
end

% release Video Player and Video Writer
release(videoPlayer);
release(videoFWriter);