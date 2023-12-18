%% Plot of heart with smiley face for extra credit

% plot heart
t = linspace (-pi,pi, 25); 
x = 16*sin (t).^3; 
y = 13*cos (t) - 5*cos (2*t) - 2*cos (3*t) - cos (4*t); 
scatter(x,y,'r')
axis equal
xlim ([-20 20]); ylim ([-20 20])
hold on

% plot eyes
eyeCenters = [-4 2; 4 2]; 
eyeRadii = [2 2]; 
n = 8;
x1 = zeros(1,n); y1 = zeros(1,n);
x2 = zeros(1,n); y2 = zeros(1,n);
for i = 1:n
    theta = 2*pi*i/n;
    x1(i) = eyeCenters(1,1) + eyeRadii(1)*cos(theta);
    y1(i) = eyeCenters(1,2) + eyeRadii(1)*sin(theta);
    x2(i) = eyeCenters(2,1) + eyeRadii(2)*cos(theta);
    y2(i) = eyeCenters(2,2) + eyeRadii(2)*sin(theta);
end
scatter(x1,y1,'k')
scatter(x2,y2,'k')


% plot mouth
mouthCenter = [0 -3];
mouthRadius = 5;
n = 6;
x3 = zeros(1,n); y3 = zeros(1,n);
for i = 1:n
    theta = pi + pi*i/n;
    x3(i) = mouthCenter(1) + mouthRadius*cos(theta);
    y3(i) = mouthCenter(2) + mouthRadius*sin(theta);
end
scatter(x3,y3,'k')

hold off;












