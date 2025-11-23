
%% Circular trajectory input for dead reckoning
x = [0; 0; 0];
v = 2;
w = 0;
dt = 1;
u = [v; w];                % your commanded v,w

X = x;
U_dead = []; 
X_dead = x;
% Go straight
for i = 1:4

    x = GTdeadReckoningMotionModel(x, u, dt);
    U_dead = [U_dead, u];
    X_dead = [X_dead, x];
end

% Turn 90 degree
v = 0;
w = pi/2;
dt = 1;
u = [v; w];                % your commanded v,w

x = GTdeadReckoningMotionModel(x, u, dt);
U_dead = [U_dead, u];
X_dead = [X_dead, x];

% Go straight again
v = 2;
w = 0;
dt = 1;
u = [v; w]; 

for i = 1:4
    
    x = GTdeadReckoningMotionModel(x, u, dt);
    U_dead = [U_dead, u];
    X_dead = [X_dead, x];
end

% Turn 90 degree again
v = 0;
w = pi/2;
dt = 1;
u = [v; w];                % your commanded v,w

x = GTdeadReckoningMotionModel(x, u, dt);
U_dead = [U_dead, u];
X_dead = [X_dead, x];

% Go straight again
v = 2;
w = 0;
dt = 1;
u = [v; w]; 

for i = 1:4
    
    x = GTdeadReckoningMotionModel(x, u, dt);
    U_dead = [U_dead, u];
    X_dead = [X_dead, x];
end

%% Circular trajectory input for odometry
x = [0; 0; 0];

X_odom = x;
U_odom = [];
for i = 1:length(U_dead(1,:))

    u = [X_dead(:,i);  X_dead(:,i+1)];
    x = GTodometryMotionModel(x, u);

    U_odom = [U_odom, u];
    X_odom = [X_odom, x];

end


%% Circular trajectory for dead reckoning model
close all

x = [0; 0; 0];
size_U = length(U_dead(1,:));

alpha = 0.001*[0.1 0.1 0.1 0.1 0.01 0.01]';   % tune these
sample_size = 100;
% X_dead_sample = concta mxNxtime;
X_dead_sample = repmat(x,1,sample_size,size_U+1);

for i=1:size_U
    
    for sample=1:sample_size
        X_dead_sample(:,sample,i+1) = sampleDeadReckoningMotionModel(X_dead_sample(:,sample,i), U_dead(:,i), alpha, dt);
    end
end

xpos = X_dead_sample(1,:,:);
xpos = xpos(:,:);
ypos = X_dead_sample(2,:,:);
ypos = ypos(:,:);

title('dead reckoning')
plot(xpos,ypos,'k.')
daspect([1,1,1])
pbaspect([1,1,1])

%% Circular trajectory for dead reckoning model
close all

x = [0; 0; 0];
size_U = length(U_odom(1,:));

alpha = 0.001*[0.1 0.1 0.1 0.1]';  % tune as needed
sample_size = 100;
% X_dead_sample = concta mxNxtime;
X_odom_sample = repmat(x,1,sample_size,size_U+1);

for i=1:size_U
    a = 3;
    for sample=1:sample_size
        X_odom_sample(:,sample,i+1) = sampleOdometryMotionModel(X_odom_sample(:,sample,i), U_odom(:,i), alpha);
    end
end

xpos = X_odom_sample(1,:,:);
xpos = xpos(:,:);
ypos = X_odom_sample(2,:,:);
ypos = ypos(:,:);

title('odometry')
plot(xpos,ypos,'k.')
daspect([1,1,1])
pbaspect([1,1,1])

