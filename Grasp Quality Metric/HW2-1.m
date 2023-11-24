%% VBM Assignment 2 
% Upasana Mahanti
clc;
clear all;
close all;

c5_positions = [];

%contact points
c1=[-3 2]; 
c2=[-1 3];
c3=[1 3];
c4=[0 0];
c5=[3 3.5];

N=[0 0]; % gripper frame 
O=[0 1.5]; % object location

CiO=c1-O;
S=[-CiO(2) CiO(1)];
St=S.';

R_c1N=[1 0;0 1];
R_c2N=[0 -1;-1 0];
R_c3N=[0 -1;-1 0];
R_c4N=[1 0;0 1];
R_c5N=[-1 0;0 1];

Gt_1=partial_grasp_matrix(R_c1N,O,c1);
Gt_2=partial_grasp_matrix(R_c2N,O,c2);
Gt_3=partial_grasp_matrix(R_c3N,O,c3);
Gt_4=partial_grasp_matrix(R_c4N,O,c4);
disp('Partial Grasp matrix of 4 points:')
disp([Gt_1;Gt_2;Gt_3;Gt_4]);
H1=[1 0 0;
    0 1 0;
    0 0 1;];
H=blkdiag(H1,H1,H1,H1,H1);
%Top surface
xt = linspace(-3,3,60);
yt = 3;  
Yt = 3*ones(1,60); 
%Bottom surface
xb = linspace(-3,3,60);
yb = 0; 
Yb = zeros(1,60);  % Vector of zeros for bottom y-coordinate
%left surface
yl = linspace(0,3,60);  
xl = -3;
Xl = -3*ones(1,60);
%right surface
yr = linspace(0,3,60); 
xr = 3;
Xr = 3*ones(1,60);
QMSV = zeros(1,30);
QVEW = zeros(1,30);
QGII = zeros(1,30);
c5_x_values = [];


for i = 1 : length(yr)
 
 R=R_c2N;
 c5=[xr yr(i)];
 c5_positions = [c5_positions; c5];

 Gt_5=partial_grasp_matrix(R,O,c5);

 Gt=[Gt_1;Gt_2;Gt_3;Gt_4;Gt_5];
 Gt=H*Gt;
 G=Gt.';

 % 3.1.1 Minimum singular value
 Singular_values = gsqrt(eig(G*Gt));
 Qmsv = min(Singular_values);
 QMSV(i)=Qmsv;
 
 % 3.1.2 Volume of the ellipsoid in the wrench space
 Qvew=sqrt(det(G*Gt));
 QVEW(i)=Qvew;
 

 % 3.1.3 Grasp isotropy index
 Qgii = min(Singular_values)/max(Singular_values);
 QGII(i)=Qgii;
 
end 
%Qmsv_value(Xr,yr,QMSV,QVEW,QGII,1);
%Qvew_value(Xr,yr,QMSV,QVEW,QGII,1);
Qgii_value(Xr,yr,QMSV,QVEW,QGII,1);

for i = 1 : length(yl)
 R=R_c2N;
 c5=[xl yl(i)];
 Gt_5=partial_grasp_matrix(R,O,c5);
 Gt=[Gt_1;Gt_2;Gt_3;Gt_4;Gt_5];
 Gt=H*Gt;
 G=Gt.';
 Singular_values = gsqrt(eig(G*Gt));
 Qmsv = min(Singular_values);
 QMSV(i)=Qmsv;
 Qvew=sqrt(det(G*Gt));
 QVEW(i)=Qvew;
 Qgii = min(Singular_values)/max(Singular_values);
 QGII(i)=Qgii;
end 
%Qmsv_value(Xl,yl,QMSV,QVEW,QGII,2);
%Qvew_value(Xl,yl,QMSV,QVEW,QGII,2);
Qgii_value(Xl,yl,QMSV,QVEW,QGII,2);

QMSV = zeros(1,60);
QVEW = zeros(1,60);
QGII = zeros(1,60);
for i = 1 : length(xt)
 R=R_c2N;
 c5=[xt(i) yt];
 Gt_5=partial_grasp_matrix(R,O,c5);
 Gt=[Gt_1;Gt_2;Gt_3;Gt_4;Gt_5];
 Gt=H*Gt;
 G=Gt.';
 Singular_values = gsqrt(eig(G*Gt));
 Qmsv = min(Singular_values);
 QMSV(i)=Qmsv;
 Qvew=sqrt(det(G*Gt));
 QVEW(i)=Qvew;
 Qgii = min(Singular_values)/max(Singular_values);
 QGII(i)=Qgii;
end 
%Qmsv_value(xt,Yt,QMSV,QVEW,QGII,3);
%Qvew_value(xt,Yt,QMSV,QVEW,QGII,3);
Qgii_value(xt,Yt,QMSV,QVEW,QGII,3);

for i = 1 : length(xb)
 R=R_c2N;
 c5=[xb(i) yb];
 Gt_5=partial_grasp_matrix(R,O,c5);
 Gt=[Gt_1;Gt_2;Gt_3;Gt_4;Gt_5];
 Gt=H*Gt;
 G=Gt.';
 Singular_values = gsqrt(eig(G*Gt));
 Qmsv = min(Singular_values);
 QMSV(i)=Qmsv;
 Qvew=sqrt(det(G*Gt));
 QVEW(i)=Qvew;
 Qgii = min(Singular_values)/max(Singular_values);
 QGII(i)=Qgii;
end 
%Qmsv_value(xb,Yb,QMSV,QVEW,QGII,4)
%Qvew_value(xb,Yb,QMSV,QVEW,QGII,4)
Qgii_value(xb,Yb,QMSV,QVEW,QGII,4)


%Partial grasp matrix
function Gt_i=partial_grasp_matrix(R_ciN,O,ci)
cio=ci-O;
S=[-cio(2) cio(1)];
St=S.';

R_ciN_bar= blkdiag(R_ciN,1);
R_Nci_bar=R_ciN_bar.';
Pi=[1 0 St(1) ;
    0 1 St(2);
    0 0  1;];
Gt_i=R_Nci_bar*Pi;

end

%% Plotting functions

function Qmsv_value(Xr, yr, QMSV, QVEW, QGII, sel)

   % Plot all the QMSV values
   subplot(1, 2, 1);
   rectangle('Position',[-3 0 6 3], 'EdgeColor', [0 0 0]);
   axis([-7 7 -7 7])
   title('All QMSV values')
   xlabel('x') 
   ylabel('y') 
   grid on;
   [peakVal, idxVal] = max(QMSV);

    switch sel
       case 1 % Right edge
           for i = 1:length(Xr) 
               x = [Xr(i) Xr(i)+QMSV(i)];
               y = [yr(i) yr(i)];
               line(x, y);
           end
           disp(['Best QMSV at Right Edge: ' num2str(peakVal)]);
       case 2 % Left edge
           for i = 1:length(Xr) 
               x = [Xr(i) Xr(i)-QMSV(i)];
               y = [yr(i) yr(i)];
               line(x, y);
           end 
           disp(['Best QMSV at Left Edge: ' num2str(peakVal)]);
       case 3 % Top edge
           for i = 1:length(Xr) 
               x = [Xr(i) Xr(i)];
               y = [yr(i) yr(i)+QMSV(i)];
               line(x, y);
           end
           disp(['Best QMSV at Top Edge: ' num2str(peakVal)]);
       case 4 % Bottom edge
           for i = 1:length(Xr) 
               x = [Xr(i) Xr(i)];
               y = [yr(i) yr(i)-QMSV(i)];
               line(x, y);
           end
           disp(['Best QMSV at Bottom Edge: ' num2str(peakVal)]);
   end

   % Highlight the best QMSV value in red
   subplot(1, 2, 2);
   rectangle('Position',[-3 0 6 3], 'EdgeColor', [0 0 0]);
   axis([-7 7 -7 7])
   title('Highlighting Best QMSV')
   xlabel('x') 
   ylabel('y') 
   grid on;

   [peakVal, idxVal] = max(QMSV);

   switch sel
       case 1 % Right edge
           x = [Xr(idxVal) Xr(idxVal)+peakVal];
           y = [yr(idxVal) yr(idxVal)];
           line(x, y, 'Color', 'red', 'LineWidth', 2);
       case 2 % Left edge
           x = [Xr(idxVal) Xr(idxVal)-peakVal];
           y = [yr(idxVal) yr(idxVal)];
           
       case 3 % Top edge
           x = [Xr(idxVal) Xr(idxVal)];
           y = [yr(idxVal) yr(idxVal)+peakVal];
           
       case 4 % Bottom edge
           x = [Xr(idxVal) Xr(idxVal)];
           y = [yr(idxVal) yr(idxVal)-peakVal];
           line(x, y, 'Color', 'red', 'LineWidth', 2);
   end

end


function Qvew_value(Xr, yr, QMSV, QVEW, QGII, sel)

   % Plot all the QVEW values
   subplot(1, 2, 1);
   rectangle('Position',[-3 0 6 3], 'EdgeColor', [0 0 0]);
   axis([-35 35 -35 35]);
   title('All QVEW values');
   xlabel('x'); 
   ylabel('y'); 
   grid on;

   [peakVal, idxVal] = max(QVEW);

   switch sel
       case 1 % Right edge
           for i = 1:length(Xr)
               x = [Xr(i) Xr(i)+QVEW(i)];
               y = [yr(i) yr(i)];
               line(x, y);
           end
           disp(['Best QVEW at Right Edge: ' num2str(peakVal)]);
       case 2 % Left edge
           for i = 1:length(Xr)
               x = [Xr(i) Xr(i)-QVEW(i)];
               y = [yr(i) yr(i)];
               line(x, y);
           end
           disp(['Best QVEW at Left Edge: ' num2str(peakVal)]);
       case 3 % Top edge
           for i = 1:length(Xr)
               x = [Xr(i) Xr(i)];
               y = [yr(i) yr(i)+QVEW(i)];
               line(x, y);
           end
           disp(['Best QVEW at Top Edge: ' num2str(peakVal)]);
       case 4 % Bottom edge
           for i = 1:length(Xr)
               x = [Xr(i) Xr(i)];
               y = [yr(i) yr(i)-QVEW(i)];
               line(x, y);
           end
           disp(['Best QVEW at Bottom Edge: ' num2str(peakVal)]);
   end

   % Highlight the best QVEW value in red
   subplot(1, 2, 2);
   rectangle('Position',[-3 0 6 3], 'EdgeColor', [0 0 0]);
   axis([-35 35 -35 35]);
   title('Highlighting Best Qvew');
   xlabel('x'); 
   ylabel('y'); 
   grid on;

   switch sel
       case 1 % Right edge
           x = [Xr(idxVal) Xr(idxVal)+peakVal];
           y = [yr(idxVal) yr(idxVal)];
           line(x, y, 'Color', 'red', 'LineWidth', 2);
       case 2 % Left edge
           x = [Xr(idxVal) Xr(idxVal)-peakVal];
           y = [yr(idxVal) yr(idxVal)];
           
       case 3 % Top edge
           x = [Xr(idxVal) Xr(idxVal)];
           y = [yr(idxVal) yr(idxVal)+peakVal];
           
       case 4 % Bottom edge
           x = [Xr(idxVal) Xr(idxVal)];
           y = [yr(idxVal) yr(idxVal)-peakVal];
           line(x, y, 'Color', 'red', 'LineWidth', 2);
   end

end


function Qgii_value(Xr, yr, QMSV, QVEW, QGII, sel)

   % Plot all the QGII values
   subplot(1, 2, 1);
   rectangle('Position',[-3 0 6 3], 'EdgeColor', [0 0 0]);
   axis([-5 5 -5 5]);
   title('All QGII values');
   xlabel('x'); 
   ylabel('y'); 
   grid on;
   [peakVal, idxVal] = max(QGII);

   switch sel
       case 1 % Right edge
           for i = 1:length(Xr)
               x = [Xr(i) Xr(i)+QGII(i)];
               y = [yr(i) yr(i)];
               line(x, y);
           end
           disp(['Best QGII at Right Edge: ' num2str(peakVal)]);
       case 2 % Left edge
           for i = 1:length(Xr)
               x = [Xr(i) Xr(i)-QGII(i)];
               y = [yr(i) yr(i)];
               line(x, y);
           end
           disp(['Best QGII at Left Edge: ' num2str(peakVal)]);
       case 3 % Top edge
           for i = 1:length(Xr)
               x = [Xr(i) Xr(i)];
               y = [yr(i) yr(i)+QGII(i)];
               line(x, y);
           end
           disp(['Best QGII at Top Edge: ' num2str(peakVal)]);
       case 4 % Bottom edge
           for i = 1:length(Xr)
               x = [Xr(i) Xr(i)];
               y = [yr(i) yr(i)-QGII(i)];
               line(x, y);
           end
           disp(['Best QGII at Bottom Edge: ' num2str(peakVal)]);
   end

   % Highlight the best QGII value in red
   subplot(1, 2, 2);
   rectangle('Position',[-3 0 6 3], 'EdgeColor', [0 0 0]);
   axis([-5 5 -5 5]);
   title('Highlighting Best QGII');
   xlabel('x'); 
   ylabel('y'); 
   grid on;

   switch sel
       case 1 % Right edge
           x = [Xr(idxVal) Xr(idxVal)+peakVal];
           y = [yr(idxVal) yr(idxVal)];
           
       case 2 % Left edge
           x = [Xr(idxVal) Xr(idxVal)-peakVal];
           y = [yr(idxVal) yr(idxVal)];
           
       case 3 % Top edge
           x = [Xr(idxVal) Xr(idxVal)];
           y = [yr(idxVal) yr(idxVal)+peakVal];
           
       case 4 % Bottom edge
           x = [Xr(idxVal) Xr(idxVal)];
           y = [yr(idxVal) yr(idxVal)-peakVal];
           line(x, y, 'Color', 'red', 'LineWidth', 2);
   end

end



%% Note
% Uncomment 79,98,120,139 lines to get Qmsv plot
% Uncomment 80,99,121,140 lines to get Qvew plot
% Uncomment 81,100,122,141 lines to get Qgii plot









