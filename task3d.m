clc;clear

%% Define variables
str = {'1st', '2nd', '3rd'};

v = [0 0 1
     0 1 0
     1 0 0];
 
w = [1 0 0
     0 1 0
     0 0 1];

theta = zeros(3,1); 
 
%% Generate rigid body motion matrix and calculate Xi.
for i = 1:3
     theta(i) = pi/(2*i);
     g(1:3, 1:3) = SKEW3(w(:, i));
     g(1:3, 4) = v(:, i);
     g(4, :) = zeros(1, 4);
     g = g.*theta(i);
     G{i} = expm(g);

     [Xi, theta_2] = getXi(G{i});
     
%% Compare the value obtained from the 'getXi' function with the initial values.
     if round(Xi(1:3), 14) == round(v(:,i)*theta(i), 14)
         if round(Xi(4:6), 14) == round(w(:,i)*theta(i), 14)
            disp(['The ', char(str(i)), ' un-normalized Xi is:']);
            disp(round(Xi,2));
            disp(['The ', char(str(i)),' theta is ', num2str(theta_2)])
            disp(' ')
         end
     end

end
