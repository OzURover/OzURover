clc
clear
close all

% Link lenghts
d0 = 6.9;
d1 = 35.6;
d2 = 31.5;
d3 = 6.7;
d4 = 13.75;

q0 = 0.000001;
q1 = 0.000001;
q2 = 0.000001;
q3 = 0.000001;


% Inverse of jacobian with init values
Jinv = [ -sin(q0)/(d2*sin(q1 + q2) + d1*sin(q1) + d3*sin(q1 + q2 + q3) + d4*sin(q1 + q2 + q3)), cos(q0)/(d2*sin(q1 + q2) + d1*sin(q1) + d3*sin(q1 + q2 + q3) + d4*sin(q1 + q2 + q3)),                                               0,                                                                        0,                                                                       0;
                                                   (sin(q1 + q2)*cos(q0))/(d1*sin(q2)),                                                  (sin(q1 + q2)*sin(q0))/(d1*sin(q2)),                       cos(q1 + q2)/(d1*sin(q2)),                                -(sin(q0)*sin(q3)*(d3 + d4))/(d1*sin(q2)),                                (cos(q0)*sin(q3)*(d3 + d4))/(d1*sin(q2));
                             -(cos(q0)*(d2*sin(q1 + q2) + d1*sin(q1)))/(d1*d2*sin(q2)),                            -(sin(q0)*(d2*sin(q1 + q2) + d1*sin(q1)))/(d1*d2*sin(q2)), -(d2*cos(q1 + q2) + d1*cos(q1))/(d1*d2*sin(q2)),       (sin(q0)*(d3 + d4)*(d1*sin(q2 + q3) + d2*sin(q3)))/(d1*d2*sin(q2)),     -(cos(q0)*(d3 + d4)*(d1*sin(q2 + q3) + d2*sin(q3)))/(d1*d2*sin(q2));
                                                        (cos(q0)*sin(q1))/(d2*sin(q2)),                                                       (sin(q0)*sin(q1))/(d2*sin(q2)),                            cos(q1)/(d2*sin(q2)), -(sin(q0)*(d3*sin(q2 + q3) + d4*sin(q2 + q3) + d2*sin(q2)))/(d2*sin(q2)), (cos(q0)*(d3*sin(q2 + q3) + d4*sin(q2 + q3) + d2*sin(q2)))/(d2*sin(q2));
                                                                                     0,                                                                                    0,                                               0,                                                cos(q0)/sin(q1 + q2 + q3),                                               sin(q0)/sin(q1 + q2 + q3)];



% Initial values of Cartesian and Joint space (xk for X(k), xko for X(k-1))
xko = [0;0;0;0;0];
qko = [0.0;90*pi/180;0.0;0.0];0.0;

%Simulaiton parameters
tstart = 0.0;
tend = 10;
tsim = tend - tstart;
dt = 0.001;
Dn = tsim/dt + 1;

DataStart = tstart/dt + 1;
temp = DataStart-1;
sstarted = tic;
for(i=1:Dn)
    
    t(i,1) = tstart + (i-1)*dt;

    % Trajectory vector
    xk = [Xe(i,1);Ye(i,1);Ze(i,1)];
    
    %Jacobian inverse kinematics
    qk = qko + Jinv*(xk-xko);
    
    % Assigning new joint values
    q0(i,1) = qk(1);
    q1(i,1) = qk(2);
    q2(i,1) = qk(3);
    q3(i,1) = qk(4);
    
    % Again, Jacobian for loop
  
Jinv = [ -sin(q0(i,1))/(d2*sin(q1(i,1) + q2(i,1)) + d1*sin(q1(i,1)) + d3*sin(q1(i,1) + q2(i,1) + q3(i,1)) + d4*sin(q1(i,1) + q2(i,1) + q3(i,1))), cos(q0(i,1))/(d2*sin(q1(i,1) + q2(i,1)) + d1*sin(q1(i,1)) + d3*sin(q1(i,1) + q2(i,1) + q3(i,1)) + d4*sin(q1(i,1) + q2(i,1) + q3(i,1))),                                               0,                                                                        0,                                                                       0;
                                                   (sin(q1(i,1) + q2(i,1))*cos(q0(i,1)))/(d1*sin(q2(i,1))),                                                  (sin(q1(i,1) + q2(i,1))*sin(q0(i,1)))/(d1*sin(q2(i,1))),                       cos(q1(i,1) + q2(i,1))/(d1*sin(q2(i,1))),                                -(sin(q0(i,1))*sin(q3(i,1))*(d3 + d4))/(d1*sin(q2(i,1))),                                (cos(q0(i,1))*sin(q3(i,1))*(d3 + d4))/(d1*sin(q2(i,1)));
                             -(cos(q0(i,1))*(d2*sin(q1(i,1) + q2(i,1)) + d1*sin(q1(i,1))))/(d1*d2*sin(q2(i,1))),                            -(sin(q0(i,1))*(d2*sin(q1(i,1) + q2(i,1)) + d1*sin(q1(i,1))))/(d1*d2*sin(q2(i,1))), -(d2*cos(q1(i,1) + q2(i,1)) + d1*cos(q1(i,1)))/(d1*d2*sin(q2(i,1))),       (sin(q0(i,1))*(d3 + d4)*(d1*sin(q2(i,1) + q3(i,1)) + d2*sin(q3(i,1))))/(d1*d2*sin(q2(i,1))),     -(cos(q0(i,1))*(d3 + d4)*(d1*sin(q2(i,1) + q3(i,1)) + d2*sin(q3(i,1))))/(d1*d2*sin(q2(i,1)));
                                                        (cos(q0(i,1))*sin(q1(i,1)))/(d2*sin(q2(i,1))),                                                       (sin(q0(i,1))*sin(q1(i,1)))/(d2*sin(q2(i,1))),                            cos(q1(i,1))/(d2*sin(q2(i,1))), -(sin(q0(i,1))*(d3*sin(q2(i,1) + q3(i,1)) + d4*sin(q2(i,1) + q3(i,1)) + d2*sin(q2(i,1))))/(d2*sin(q2(i,1))), (cos(q0(i,1))*(d3*sin(q2(i,1) + q3(i,1)) + d4*sin(q2(i,1) + q3(i,1)) + d2*sin(q2(i,1))))/(d2*sin(q2(i,1)));
                                                                                     0,                                                                                    0,                                               0,                                                cos(q0(i,1))/sin(q1(i,1) + q2(i,1) + q3(i,1)),                                               sin(q0(i,1))/sin(q1(i,1) + q2(i,1) + q3(i,1))];

    %Assigning new values as previous values.
    qko = qk;
    xko = xk;
    
% Forward Kinematics for simulation    

    rf_P0(i-temp,1) = 0;
    rf_P0(i-temp,2) = 0;
    rf_P0(i-temp,3) = 0;

    rf_P1(i-temp,1) =  P1(1);
    rf_P1(i-temp,2) =  P1(2);
    rf_P1(i-temp,3) =  P1(3);
  
    rf_P2(i-temp,1) = P2(1);
    rf_P2(i-temp,2) = P2(2);
    rf_P2(i-temp,3) = P2(3);

    rf_P3(i-temp,1) =  P3(1);
    rf_P3(i-temp,2) =  P3(2);
    rf_P3(i-temp,3) =  P3(3);    

    rf_Pf(i-temp,1) = Pf(1);
    rf_Pf(i-temp,2) = Pf(2);
    rf_Pf(i-temp,3) = Pf(3); 
    
     
end
tElapsed = toc(sstarted);

figure(1);
set(gcf,'Renderer','OpenGL');
set(gcf, 'Position', [300 200 1200 600]);
RF_P0 = plot3(rf_P0(1,1),rf_P0(1,2),rf_P0(1,3),'o','MarkerSize',10,'MarkerFaceColor','g');
hold on;
RF_P1 = plot3(rf_P1(1,1),rf_P1(1,2),rf_P1(1,3),'o','MarkerSize',10,'MarkerFaceColor','b');
RF_P2 = plot3(rf_P2(1,1),rf_P2(1,2),rf_P2(1,3),'o','MarkerSize',10,'MarkerFaceColor','r');
RF_P3 = plot3(rf_P3(1,1),rf_P3(1,2),rf_P3(1,3),'o','MarkerSize',10,'MarkerFaceColor','m');
RF_Pf = plot3(rf_Pf(1,1),rf_Pf(1,2),rf_Pf(1,3),'o','MarkerSize',10,'MarkerFaceColor','m');

RF_D0 = plot3([rf_P0(1,1) rf_P1(1,1)], [rf_P0(1,2) rf_P1(1,2)], [rf_P0(1,3) rf_P1(1,3)],'LineWidth',4,'Color','k');
RF_D1 = plot3([rf_P1(1,1) rf_P2(1,1)], [rf_P1(1,2) rf_P2(1,2)], [rf_P1(1,3) rf_P2(1,3)],'LineWidth',4,'Color','k');
RF_D2 = plot3([rf_P2(1,1) rf_P3(1,1)], [rf_P2(1,2) rf_P3(1,2)], [rf_P2(1,3) rf_P3(1,3)],'LineWidth',4,'Color','k');
RF_D3 = plot3([rf_P3(1,1) rf_Pf(1,1)], [rf_P3(1,2) rf_Pf(1,2)], [rf_P3(1,3) rf_Pf(1,3)],'LineWidth',4,'Color','k');
% L2 = plot3([rf_b(1,1) rf_c(1,1)], [0,0],[rf_b(1,2) rf_c(1,2)],'LineWidth',4,'Color','k');

xlim([-0.3 0.3]);xlabel('X[m]','FontSize',16);
ylim([-0.3 0.3]);ylabel('Y[m]','FontSize',16);
zlim([-1 0]);zlabel('Z[m]','FontSize',16);
grid;
    
set(RF_P0,'EraseMode','normal');
set(RF_P1,'EraseMode','normal');
set(RF_P2,'EraseMode','normal');
set(RF_P3,'EraseMode','normal');
set(RF_Pf,'EraseMode','normal');
set(RF_D0,'EraseMode','normal');
set(RF_D1,'EraseMode','normal');
set(RF_D2,'EraseMode','normal');
set(RF_D3,'EraseMode','normal');

i = 1;
while i<=size(rf_P0,1)
    set(RF_P0,'XData',rf_P0(i,1),'YData',rf_P0(i,2),'ZData',rf_P0(i,3));
    set(RF_P1,'XData',rf_P1(i,1),'YData',rf_P1(i,2),'ZData',rf_P1(i,3));
    set(RF_P2,'XData',rf_P2(i,1),'YData',rf_P2(i,2),'ZData',rf_P2(i,3));
    set(RF_P3,'XData',rf_P3(i,1),'YData',rf_P3(i,2),'ZData',rf_P3(i,3));
    set(RF_Pf,'XData',rf_Pf(i,1),'YData',rf_Pf(i,2),'ZData',rf_Pf(i,3));
    
    set(RF_D0,'XData',[rf_P0(i,1) rf_P1(i,1)],'YData',[rf_P0(i,2) rf_P1(i,2)],'ZData',[rf_P0(i,3) rf_P1(i,3)]);
    set(RF_D1,'XData',[rf_P1(i,1) rf_P2(i,1)],'YData',[rf_P1(i,2) rf_P2(i,2)],'ZData',[rf_P1(i,3) rf_P2(i,3)]);
    set(RF_D2,'XData',[rf_P2(i,1) rf_P3(i,1)],'YData',[rf_P2(i,2) rf_P3(i,2)],'ZData',[rf_P2(i,3) rf_P3(i,3)]);
    set(RF_D3,'XData',[rf_P3(i,1) rf_Pf(i,1)],'YData',[rf_P3(i,2) rf_Pf(i,2)],'ZData',[rf_P3(i,3) rf_Pf(i,3)]);
    drawnow;
    i = i+25;
end