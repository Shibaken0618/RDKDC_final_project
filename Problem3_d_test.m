clc;clear

%% Define variables
str = {'1st', '2nd', '3rd'};

v = [0 0 1
     0 1 0
     1 0 0];
 
w = [1 0 0
     0 1 0
     0 0 1];

theta_1 = zeros(3,1); 
 
%% Generate rigid body motion matrix and calculate Xi.
for i = 1:3
     theta_1(i) = pi/(2*i);
     G{i} = RigidBodyMotion(v(:, i), w(:, i), theta_1(i));
     [Xi, theta_2] = getXi(G{i});
     
%% Compare the value obtained from the 'getXi' function with the initial values.
     if round(Xi(1:3), 2) == round(v(:,i), 2)
        if round(theta_2, 2) == round(theta_1(i), 2)
            if round(Xi(4:6), 2) == round(w(:,i), 2)  %% The logical arraies are not in the same size, thus cannot be connected by '&&'
                disp(['The ', char(str(i)), ' normalized Xi is:']);
                disp(round(Xi,2));
                disp(['The ', char(str(i)),' theta is ', num2str(theta_2)])
                disp(' ')
            end
        end
     end
     
end
