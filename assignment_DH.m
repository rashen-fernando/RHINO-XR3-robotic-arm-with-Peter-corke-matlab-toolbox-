%link([theta     d        a      alpha])
L(1)=Link([0      .24765    0        pi/2]);
L(2)=Link([0       0        .2286       0]);
L(3)=Link([0      .2286     0        pi/2]);
L(4)=Link([0       0        0        pi/2]);
L(5)=Link([0      .079375   0        pi/2]);
Robot=SerialLink(L);
Robot.name="rob";
%F=Robot.fkine(qi)

%desired end effector ponts
H = SE3(-0.146, 0, 0.409) * SE3.rpy(0,-90,-180, 'deg');
%e number - 103 odd hence 
Ay=0.3; Az=.01+103*0.4/420;
A = SE3(-0.17, Ay,Az) * SE3.rpy(-180,0,60, 'deg');
B = SE3(0.181, 0.313, 0.345) * SE3.rpy(-125, 26, 106, 'deg');
C = SE3(0.420, 0.000, 0.540) * SE3.rpy( 0, 70, 0, 'deg');
D = SE3(0.237, -0.338, 0.100) * SE3.rpy( 180, 0, -125, 'deg');
% q=Robot.ikunc(H);
% Robot.plot(q);hold on;trplot(H);
Wp = [Robot.ikunc(H);Robot.ikunc(A);Robot.ikunc(A);Robot.ikunc(B);Robot.ikunc(B);Robot.ikunc(C);Robot.ikunc(C);Robot.ikunc(D);Robot.ikunc(D);Robot.ikunc(H)];
a=10;b=50;
DT=15;
QDmax= [b b b b b];
tseg = [a a a a a ];
trajectory1 = mstraj(Wp,QDmax,[],Robot.ikunc(H),DT,10);

%trplot(H);hold on;trplot(A);hold on;trplot(B);hold on;trplot(C);hold on;trplot(D);hold on;
plot3(-0.146, 0, 0.409,'*');hold on;
plot3(-0.17, Ay,Az,'o');hold on;
plot3(0.181, 0.313, 0.345,'d');hold on;
plot3(0.420, 0.000, 0.540,'p');hold on;
plot3(0.237, -0.338, 0.100,'h');hold on;


for i = 1:length(trajectory1) 
 Robot.plot(trajectory1(i,:));pause(0.25); 
end 