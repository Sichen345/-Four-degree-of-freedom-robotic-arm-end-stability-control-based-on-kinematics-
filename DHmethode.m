function T=DHmethode(a,alpha,d,theta)

%Translation sur x

T1 = [1 0 0 a
      0 1 0 0
      0 0 1 0
      0 0 0 1];
  
%Rotation sur x

T2 = [1     0           0                0
      0     cos(alpha) -sin(alpha)       0
      0     sin(alpha)  cos(alpha)       0
      0     0           0                1];


%Translation sur z

T3 = [1 0 0 0
      0 1 0 0
      0 0 1 d
      0 0 0 1];
  
%Rotation sur z

T4 = [cos(theta)     -sin(theta)      0       0
      sin(theta)     cos(theta)       0       0
      0              0                1       0
      0              0                0       1];
  
%Matrice globale

T=T1*T2*T3*T4;

end