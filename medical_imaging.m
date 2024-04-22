close all; clear all %% phantom image 

P=dicomread("IMAGE1_90kV");
figure(1),imshow(P,[])
title('Original Image')

% Compute fan-beam projection data of the phantom image
R = 600; %% R > = image size / 2

%% Parameters

FanSensorGeometry1='arc'; % 'arc' and 'line'
FanSensorSpacing1=0.2; %, the arc of sensors is fixed = 87.7 degrees, Spacing (Δr): 0.85°, 0.55°, 0.35°, 0.25°, 0.15°
FanRotationIncrement1=1; % Increment (Δs): 1
method_inter='linear'; % Interpolation: 'nearest', 'linear', 'cubic', 'spline' 
filter='Hamming'; % Filter: 'Ram-Lak','Shepp-Logan','Cosine','Hamming','Hann# 
filter_cutoff=0.8; %% cut-off frequency: 95%, 85%, 65%, 45%, 25% of window 

[F3, sensor_pos, fan_rot_angle] = fanbeam(P,R,'FanRotationIncrement',FanRotationIncrement1, 'FanSensorGeometry',FanSensorGeometry1,'FanSensorSpacing',FanSensorSpacing1); 

%% Plot the projection data 

figure (2), imagesc(fan_rot_angle, sensor_pos, F3)
colormap(hot); colorbar
xlabel('Fan Rotation Angle (degrees)')
ylabel('Fan Sensor Position (degrees)') 
title(['\Delta\theta = ',num2str(FanRotationIncrement1), ' & \Deltar = ', num2str(FanSensorSpacing1)])

% % Reconstruct the image from the fan-beam projection data using ifanbeam. 
% In each reconstruction:
output_size = max(size(P));
FanCoverage1='cycle'; %'minimal', 'cycle', no difference exists 

Ifan3=ifanbeam(F3,R,'FanRotationIncrement',FanRotationIncrement1, 'FanSensorGeometry',FanSensorGeometry1, 'FanSensorSpacing',FanSensorSpacing1,'Filter',filter,'FrequencyScaling',filter_cutoff,'Interpolation',method_inter,'OutputSize',output_size); 
figure (3), imshow(Ifan3,[]) 
%figure;imagesc(Ifan3);colormap(gray);
title(['filter:',num2str(filter), ' & cutoff freq: ',num2str(filter_cutoff)])

yi=[84,309]; 
xi=[262,262];
c = improfile(Ifan3,xi,yi);

%% Noise

%figure(3)
figure(1)
% Select ROIs on the original image
disp('Select ROIs on the reconstructed image :');
rois = drawcircle('Color','m','MarkerSize',1);
roi2 = drawcircle('Color','y','MarkerSize',1);

% Measure background noise magenda
backgroundMask = createMask(rois);
%backgroundPixels = Ifan3(backgroundMask);
backgroundPixels = P(backgroundMask);
backgroundNoise = std(double(backgroundPixels(:)));
backgroundmean = mean(double(backgroundPixels));

% Measure background noise 2 yellow
backgroundMask2 = createMask(roi2);
%backgroundPixels2 = Ifan3(backgroundMask2);
backgroundPixels2 = P(backgroundMask2);
backgroundNoise2 = std(double(backgroundPixels2(:)));
backgroundmean2 = mean(double(backgroundPixels2));

%figure(3);
figure(1);
text(10, 20, ['Backround Noise ROI_1: ', num2str(backgroundNoise)], 'Color', 'm', 'FontSize', 14);
text(10, 50, ['Backround Noise ROI_2: ', num2str(backgroundNoise2)], 'Color', 'y', 'FontSize', 14);


%% CNR

backnoise=(backgroundNoise+backgroundNoise2)/2;
backmean = (backgroundmean+backgroundmean2)/2;

%figure(4), imshow(Ifan3,[]) 
figure(4), imshow(P,[]) 
title(['filter:',num2str(filter), ' & cutoff freq: ',num2str(filter_cutoff)])
disp('Select ROIs on the reconstructed image :');

obj_roi1 = drawcircle('Color',[0.4940 0.1840 0.5560],'MarkerSize',1);
obj_roi2 = drawcircle('Color',[0.9290 0.6940 0.1250],'MarkerSize',1);

objectMask1 = createMask(obj_roi1);
objectPixels1 = P(objectMask1);
objectMean1 = mean(double(objectPixels1(:)));
cnr1 = abs((objectMean1-backmean)/backnoise);

objectMask2 = createMask(obj_roi2);
objectPixels2 = P(objectMask2);
objectMean2 = mean(double(objectPixels2(:)));
cnr2 = abs((objectMean2-backmean)/backnoise);

figure(4);
text(10, 20, ['CNR 1: ', num2str(cnr1)], 'Color', [0.4940 0.1840 0.5560], 'FontSize', 14);
text(10, 50, ['CNR 2: ', num2str(cnr2)], 'Color', [0.9290 0.6940 0.1250], 'FontSize', 14);

% %% Estimate Edge Spread (Blur) Function
% 
% %figure(3)
% figure(1)
% 
% l1 = drawline;
% maskline1=createMask(l1);
% %line1=Ifan3(maskline1);
% line1=P(maskline1);
% 
% l2 = drawline;
% maskline2=createMask(l2);
% %line2=Ifan3(maskline2);
% line2=P(maskline2);
% 
% figure(4)
% subplot(1,2,1)
% plot(line1)
% title("Object 1 ESF")
% subplot(1,2,2)
% plot(line2)
% title("Object 2 ESF")
% sgtitle(['filter:',num2str(filter), ' & cutoff freq: ',num2str(filter_cutoff)])

