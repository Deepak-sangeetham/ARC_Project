clc;
close all;

%joint space is the space in which the end effector of the 2R manipulator
%takes position. For a 2R manipulator, the joint space is a toroid visioned
%on a 2d plane.

l1 = 4; l2 = 2;

q1 = 0:pi/20:2*pi;
q2 = 0:pi/20:2*pi;
x = zeros(1,1681);
y = zeros(1,1681);
for i=1:length(q1)
    for j=1:length(q2)
l1x(41*(i-1)+j) = l1.*cos(q1(i));
l1y(41*(i-1)+j) = l1.*sin(q1(i));
x(41*(i-1)+j) = l1.*cos(q1(i)) + l2.*cos(q1(i) + q2(j));
y(41*(i-1)+j) = l1.*sin(q1(i)) + l2.*sin(q1(i)+q2(j));
    end
end

 xca = [-8 8];
 xcb = [0 0];
 yca = [-8 8];
 ycb = [0 0];
 figure(2);
 plot(xca,xcb,'black',ycb,yca,'black');
 hold on;

 for i=1:length(x)
    link1c1x = [0 l1x(i)];
    link2c1x = [l1x(i) x(i)];
    link1c1y = [0 l1y(i)];
    link2c1y = [l1y(i) y(i)];
    display(link1c1x);
    plot(link1c1x,link1c1y,'r',link2c1x,link2c1y,'g');
    xlabel('x');ylabel('y');
    title('Joint space of 2r manipulator');
    pause(0.075);
 end
 hold on;
 t = linspace(0,2*pi,200);
 x1 = 6*cos(t);
 y1 = 6*sin(t);
 x2 = 2*cos(t);
 y2 = 2*sin(t);
 plot(x1,y1,'b');
 hold on;
 plot(x2,y2,'b');


