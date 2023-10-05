%% Calling simulink model and security

clc
clear all

Gain = 100;
InitSpeed = 35; 
decelLimN = -200;
hdecelLimN = 1.1*decelLimN;
decelLimP = -150;
hdecelLimP = 1.1*decelLimP;

[A,B,C,D,Kess, Kr, Ke, uD] = designControl(secureRand(),Gain);
open_system('LaneMaintainSystem.slx')

set_param('LaneMaintainSystem/VehicleKinematics/Saturation','LowerLimit',num2str(decelLimN))
set_param('LaneMaintainSystem/VehicleKinematics/vx','InitialCondition',num2str(InitSpeed))

simModel = sim('LaneMaintainSystem.slx');

figure
plot(simModel.sx1.Time,simModel.sx1.Data)
title('Distance from the car')



%figure
%plot(simModel.ax1.Time,simModel.ax1.Data)
%title('Deceleration of the car')

% User profiles for HR and RR under LCW and HCW conditions
userProfiles = [
    % User LCW HCW
    % HR (mean, std) RR (mean, std) HR (mean, std) RR (mean, std)
    80, 14, 16, 6, 95, 26, 21, 16;
    65, 15, 13, 4, 71, 21, 14, 5;
    61, 14, 17, 8, 92, 23, 26, 16;
];

% Calculate the average user profile
avgUserProfile = mean(userProfiles);

% Extract the average HR and RR values for LCW and HCW
avgHR_LCW = avgUserProfile(1);
avgRR_LCW = avgUserProfile(3);
avgHR_HCW = avgUserProfile(5);
avgRR_HCW = avgUserProfile(7);
avgHR_LCW_std = avgUserProfile(2);
avgRR_LCW_std = avgUserProfile(4);
avgHR_HCW_std = avgUserProfile(6);
avgRR_HCW_std = avgUserProfile(8);

% Display the average user profile
disp('Average User Profile:');
disp(['Average HR (LCW): ', num2str(avgHR_LCW)]);
disp(['Average RR (LCW): ', num2str(avgRR_LCW)]);
disp(['Average HR (HCW): ', num2str(avgHR_HCW)]);
disp(['Average RR (HCW): ', num2str(avgRR_HCW)]);
disp(['Average HR std(LCW): ', num2str(avgHR_LCW_std)]);
disp(['Average RR std(LCW): ', num2str(avgRR_LCW_std)]);
disp(['Average HR std(HCW): ', num2str(avgHR_HCW_std)]);
disp(['Average RR std(HCW): ', num2str(avgRR_HCW_std)]);


% Create Gaussian models for HR and RR under LCW and HCW conditions
numSamples = 1000;  % Number of samples to generate

% Generate HR and RR samples for LCW and HCW using Gaussian distributions
HR_LCW_samples = normrnd(avgHR_LCW, avgHR_LCW_std, 1, numSamples);
RR_LCW_samples = normrnd(avgRR_LCW, avgRR_LCW_std, 1, numSamples);

HR_HCW_samples = normrnd(avgHR_HCW, avgHR_HCW_std, 1, numSamples);
RR_HCW_samples = normrnd(avgRR_HCW, avgRR_HCW_std, 1, numSamples);

% Calculate respiratory quotient (RQ) for LCW and HCW
RQ_LCW = HR_LCW_samples / RR_LCW_samples;
RQ_HCW = HR_HCW_samples / RR_HCW_samples;

% Calculate human reaction time (𝑡r) for LCW and HCW
tr_LCW = 0.01 * RQ_LCW;
tr_HCW = 0.01 * RQ_HCW;

% Display sample statistics
disp('Sample Statistics (LCW):');
disp(['Mean HR (LCW): ', num2str(mean(HR_LCW_samples))]);
disp(['Mean RR (LCW): ', num2str(mean(RR_LCW_samples))]);
disp(['Mean RQ (LCW): ', num2str(mean(RQ_LCW))]);
disp(['Mean 𝑡r (LCW): ', num2str(mean(tr_LCW))]);

disp('Sample Statistics (HCW):');
disp(['Mean HR (HCW): ', num2str(mean(HR_HCW_samples))]);
disp(['Mean RR (HCW): ', num2str(mean(RR_HCW_samples))]);
disp(['Mean RQ (HCW): ', num2str(mean(RQ_HCW))]);
disp(['Mean 𝑡r (HCW): ', num2str(mean(tr_HCW))]);

open_system('HumanReactionModel.slx')

set_param('HumanReactionModel/Step','Time',num2str(mean(tr_LCW)))
set_param('HumanReactionModel/Step','After',num2str(hdecelLimP))

set_param('HumanReactionModel/HumanReactionModel/Saturation','LowerLimit',num2str(hdecelLimP))
set_param('HumanReactionModel/HumanReactionModel/vx','InitialCondition',num2str(InitSpeed))

humModelP = sim('HumanReactionModel.slx');

open_system('HumanReactionModel.slx')

set_param('HumanReactionModel/Step','Time',num2str(mean(tr_HCW)))
set_param('HumanReactionModel/Step','After',num2str(hdecelLimN))

set_param('HumanReactionModel/HumanReactionModel/Saturation','LowerLimit',num2str(hdecelLimN))
set_param('HumanReactionModel/HumanReactionModel/vx','InitialCondition',num2str(InitSpeed))

humModelN = sim('HumanReactionModel.slx');




% Plot histograms of RQ and 𝑡r
%figure;
%subplot(2, 2, 1);
%histogram(RQ_LCW, 50);
%title('RQ Distribution (LCW)');
%xlabel('RQ');
%ylabel('Frequency');

%subplot(2, 2, 2);
%histogram(tr_LCW, 50);
%title('𝑡r Distribution (LCW)');
%xlabel('𝑡r');
%ylabel('Frequency');

%subplot(2, 2, 3);
%histogram(RQ_HCW, 50);
%title('RQ Distribution (HCW)');
%xlabel('RQ');
%ylabel('Frequency');

%subplot(2, 2, 4);
%histogram(tr_HCW, 50);
%title('𝑡r Distribution (HCW)');
%xlabel('𝑡r');
%ylabel('Frequency');

%advisory control:
function SWITCH = advisoryControl(vA, G, amax, tr)
    % Step 1: Predict if autonomous braking control collides or not
    predictedStoppingTime = computeStoppingTime(vA, G, amax);
    
    % Step 2: If no collision, do not switch to manual control
    if predictedStoppingTime >= tr
        SWITCH = 0; % No need to switch
        return;
    end
    
    % Step 3: Collision time 𝑡c = predicted stopping time from Step 1
    tc = predictedStoppingTime;
    
    % Step 4: Predict action time 𝑡a using constant deceleration 1.1 * amax
    ta = computeActionTime(vA, amax);
    
    % Step 5: Predict reaction time 𝑡r
    trPredicted = tr;
    
    % Step 6: Compute total time ℎstop = 𝑡r + 𝑡a
    hstop = trPredicted + ta;
    
    % Step 7: If ℎstop < 𝑡c, switch to human; otherwise, do not switch
    if hstop < tc
        SWITCH = 1; % Switch to human
    else
        SWITCH = 0; % Do not switch
    end
end

% Function to compute stopping time based on controller gain and deceleration limit
function stoppingTime = computeStoppingTime(vA, G, amax)
    % Implement your stopping time calculation logic here
    % This can involve mathematical modeling of vehicle kinematics
    % For simplicity, you can use a basic formula as an example
    stoppingTime = vA / (G * amax);
end

% Function to compute action time based on initial speed and deceleration limit
function actionTime = computeActionTime(vA, amax)
    % Implement your action time calculation logic here
    % This can involve mathematical modeling of vehicle kinematics
    % For simplicity, you can use a basic formula as an example
    actionTime = vA / (1.1 * amax);
end



