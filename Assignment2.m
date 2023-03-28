rosshutdown
ip_TurtleBot_vm = "http://192.168.29.130:11311"
rosinit(ip_TurtleBot_vm)
fprintf('-----------------------------------------------------------------\n');
fprintf('Ros Topics:\n');
rostopic list %If you do not see any topics, then the network has not been set up properly.

fprintf('-----------------------------------------------------------------\n');
fprintf('Ros Nodes:\n');
rosnode list

% Subscribe to the camera topic:
image_sub = rossubscriber('/camera/image/compressed', "sensor_msgs/CompressedImage");

% After subscribing to the camera topic, wait for the data and then display it.
image_compressed = receive(image_sub)

image_compressed.Format = 'bgr8; jpeg compressed bgr8';
figure
imshow(readImage(image_compressed));

% To continuously display updating camera images while the robot turns for
% a short duration, set the desired velocity then use the following while
% loop:
% Set a variable velocity to use for a brief TurtleBot movement.
robotCmd = rospublisher("/cmd_vel","geometry_msgs/Twist") ;
velMsg = rosmessage(robotCmd);
velocity = -0.1; % meters per second
velMsg.Angular.Z = velocity;
send(robotCmd,velMsg)
tic


while toc < 20
 image_compressed = receive(image_sub);
 image_compressed.Format = 'bgr8; jpeg compressed bgr8';
 C = readImage(image_compressed);

 bw=rgb2gray(C); %Convert color to gray scale image 
 bw_canny = edge(bw,'canny');

 [H,T,R] = hough(bw_canny,'RhoResolution',0.5,'ThetaResolution',0.5);

 numpeaks = 5; %Specify the number of peaks
 P  = houghpeaks(H,numpeaks);

 lines = houghlines(bw_canny,T,R,P,'FillGap',10,'MinLength',15);

subplot(311); imshow(C)
subplot(312); imshow(bw)
title('Original Image')
subplot(313); imshow(bw_canny)
title('Canny')
hold on

max_len = 0;
for k = 1:length(lines)
   xy = [lines(k).point1; lines(k).point2];
   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');

   % Plot beginnings and ends of lines
   plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
   plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');

   % Determine the endpoints of the longest line segment
   len = norm(lines(k).point1 - lines(k).point2);
   if ( len > max_len)
      max_len = len;
      xy_long = xy;
   end
end



%Show the rho-theta voting
imshow(imadjust(mat2gray(H)),'XData',T,'YData',R,...
      'InitialMagnification','fit');
title('Hough transform results');
xlabel('\theta'), ylabel('\rho');
axis on, axis normal, hold on;
colormap(gca,hot);

imshow(H,[],'XData',T,'YData',R,'InitialMagnification','fit');
xlabel('\theta'), ylabel('\rho');
axis on, axis normal, hold on;
plot(T(P(:,2)),R(P(:,1)),'s','color','white');

end

close all
clear
rosshutdown;
