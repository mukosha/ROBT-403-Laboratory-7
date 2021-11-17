# ROBT-403-Laboratory-7
ROBT 403 Laboratory 7

# General Goal – Obtain Inverse Kinematics without the robot model

## Matlab scripts – Download and run Matlab example. 
The example is for 5-joint planar robot arm from ROS labs.

## The lab 7 task achievements

### Parameters of the Planar Robot
![image](https://user-images.githubusercontent.com/47817099/142166860-378f63fa-3503-4e4f-851b-684bcfb24e5b.png)

### Initially the error was equal to the 33.19%:
![image](https://user-images.githubusercontent.com/47817099/142166942-637377c5-2139-4d8f-9804-7e645cd14051.png)


### After using the NN training without committing any changes to the code, I was able to obtain the error of 45.61%:

![image](https://user-images.githubusercontent.com/47817099/142167097-c309bd2a-7b1f-4e7d-ba49-3723a651bb43.png)

![image](https://user-images.githubusercontent.com/47817099/142167118-70bd4f25-04ae-4aec-bfad-4f6dadfc612e.png)


### In order to improve the NN training model, I decided that changing some layers and changing the number of maxEpochs to = 20; and miniBatchSize to = 200 should improve the results:
![image](https://user-images.githubusercontent.com/47817099/142167276-3e8f62ec-df16-4ed5-b499-5837236931a5.png)


### New NN Training:
![image](https://user-images.githubusercontent.com/47817099/142167311-1306c1db-9b10-42dc-9311-4b27669c9dbd.png)
      
### The results have improved. The new error is 27.19%: 
![image](https://user-images.githubusercontent.com/47817099/142167403-52509e5f-741e-449d-8a6f-4b4849d054c0.png)

Therefore, I can conclude that the new NN training has imporved the results greatly.

### Scatter3 to show FK resutls:
![image](https://user-images.githubusercontent.com/47817099/142167507-5ed02b21-3531-4445-a36b-cf7a9aa0f1f7.png)

![image](https://user-images.githubusercontent.com/47817099/142167521-8a5ea812-20ae-45e4-a683-f57cb8f75656.png)

## NN training model:
```script
% the data set size can be also tuned
data_size = 5000;
[XTrain, YTrain] = create_dataset(data_size);
%q = [q1, q2, q3, q4, q5];
XXy = XTrain;
% nFeatures = 20; 
% nExamples = 10000;
% 
% 
% nOutputs = 1; % this example is for setting up a regression problem
% 
% x = rand(nExamples,nFeatures); 
% t = rand(nExamples, nOutputs);

%%
XTrain = reshape(XTrain', [1, 1, size(XTrain,2),size(XTrain,1)]);


nFeatures = 3;
numClasses = 5;

layers = [ ...
    imageInputLayer([1 1 nFeatures]);
    fullyConnectedLayer(512)
    leakyReluLayer
    fullyConnectedLayer(256)
    leakyReluLayer
    fullyConnectedLayer(128)
    leakyReluLayer
    fullyConnectedLayer(64)
    leakyReluLayer
    fullyConnectedLayer(numClasses)  
    regressionLayer  
    ]
maxEpochs = 20;
miniBatchSize = 200;

options = trainingOptions('adam', ...
    'ExecutionEnvironment','cpu', ...
    'GradientThreshold',1, ...
    'MaxEpochs',maxEpochs, ...
    'MiniBatchSize',miniBatchSize, ...
    'SequenceLength','longest', ...
    'Shuffle','never', ...
    'Verbose',0, ...
    'Plots','training-progress');


net = trainNetwork(XTrain, YTrain,layers,options);
save net
```

## Demo:
```script
planarrobot_student
load net
% clear q1 q2 q3 q4 q5 forh 
%%
% rng(0,'twister');
test_size = 1000;
% q1
q_min = -114;
q_max = 114;
q1 = (q_max-q_min)*rand(test_size,1) + q_min;

% q2
q_min = -60;
q_max = 60;
q2 = (q_max-q_min)*rand(test_size,1) + q_min;

% q3
q_min = -100;
q_max = 100;
q3 = (q_max-q_min)*rand(test_size,1) + q_min;

% q4
q_min = -50;
q_max = 50;
q4 = (q_max-q_min)*rand(test_size,1) + q_min;

% q5
q_min = -50;
q_max = 50;
q5 = (q_max-q_min)*rand(test_size,1) + q_min;

forh = [q1 q2 q3 q4 q5]*deg;

% true
% home = [0 0 -0 0 0]*deg;

error = [];

for i=1:test_size
    pose_end = planar_robot.fkine(forh(i, :));
    xyz = transl(pose_end);
    
    sample = ones(1,1,3);
    sample(1,:) = xyz;

    makeq = predict(net, sample);
    pr_pose = planar_robot.fkine(makeq);
    pr_xyz = transl(pr_pose);
    
    
    error = [error, sqrt((xyz(1)-pr_xyz(1))^2 + (xyz(2)-pr_xyz(2))^2 + (xyz(3)-pr_xyz(3))^2)];
end

% predicted
%%
% this mean error should be lower than in untuned model
a = mean(error);
% home1 = [0 0 0 0 0]*deg;


forh
makeq

xyz
pr_xyz

a


home = [45 45 0 45 -45]*deg; 
planar_robot.plot(home)
pose_end2 = planar_robot.fkine(home); 
hold on

%real
xyz1 = transl(pose_end2);
scatter3(xyz1(1),xyz1(2),xyz1(3), '*')

% predicted
samplep = ones(1,1,3);
samplep(1,:) = xyz1;
xyz2 = predict(net, samplep);
predicted = [xyz2(1),xyz2(2),xyz2(3), xyz2(4), xyz2(5)]
pose_end2 = planar_robot.fkine(predicted);
xyz3 = transl(pose_end2);
hold on 
scatter3(xyz3(1),xyz3(2),xyz3(3),'filled')
```
## Thank you for your attention!
