% This is an assignment for extra credit for MAE30A
% script RR_bike_frame.m
% Structural analysis of an idealized bicycle frame, built on the HW3 carport (Renaissance Repository, https://github.com/tbewley/RR).
% 
% 2-D first, then 3-D split into Part A (TRUSS model) and Part B (FRAME model).
% Author: <Mingxiong Zhou>   Date: 2026-07-24   Repo: https://github.com/mingxiong-zhou/UCSD-work.git
%
% The following code is used to analyze the force and truss of a bicycle frame based on the MAE30A.
% set up
% geometry (A bike frame)
%   HT head tube (24,22)   ST seat tube top / saddle (6,24)   BB bottom bracket (12,6)
%   RD rear dropout (0,8)   FD front dropout (30,0)   [RD,FD are the two ground supports]
% Members: top tube HT-ST, down tube HT-BB, seat tube ST-BB, chain stay BB-RD,
%          seat stay ST-RD, fork HT-FD.
% Loads: rider weight -180 lb at the saddle (ST); pedal load -60 lb at the BB.

%% 2-D  TRUSS model (all joints pinned) 
clear S L, disp('2-D bicycle frame -- TRUSS model (two-force members)')
S.P=[  0   30;        
       8    0];
S.Q=[ 24    6   12;   
      22   24    6];
L.U=[  0    0    0;   
       0 -180  -60];
% columns of C: [HT ST BB RD FD]
S.C=[ 1  1  0  0  0;   
      1  0  1  0  0;  
      0  1  1  0  0;   
      0  0  1  1  0;   
      0  1  0  1  0;  
      1  0  0  0  1];  
[A,b,S,L]=RR_Structure_Analyze(S,L); x=pinv(A)*b;
figure(1); RR_Structure_Plot(S,L,x); error_norm=norm(A*x-b), pause

%% 2-D  FRAME model (welded main triangle) 
disp('2-D bicycle frame -- FRAME model (main triangle = rigid multiforce panel)')
S.C=[ 1  1  1  0  0;   
      0  0  1  1  0;  
      0  1  0  1  0;   
      1  0  0  0  1]; 
[A,b,S,L]=RR_Structure_Analyze(S,L); x=pinv(A)*b;
figure(2); RR_Structure_Plot(S,L,x); error_norm=norm(A*x-b), pause

%% 3-D  PART A : TRUSS model
clear S L, disp('3-D bicycle frame -- PART A -- TRUSS model')
S.P=[  0   0;         
      -2   2;
       8   8];
S.R=[ 30  30;        
      -2   2;
       0   0];
S.Q=[ 24    6   12;  
       0    0    0;
      22   24    6];
L.U=[  0    0    0;
       0    0    0;
       0 -180  -60];
% columns of C: [HT ST BB RDL RDR FDL FDR]
S.C=[ 1  1  0  0  0  0  0;  
      1  0  1  0  0  0  0; 
      0  1  1  0  0  0  0; 
      0  0  1  1  0  0  0;
      0  0  1  0  1  0  0;
      0  1  0  1  0  0  0; 
      0  1  0  0  1  0  0; 
      1  0  0  0  0  1  0;
      1  0  0  0  0  0  1]; 
[A,b,S,L]=RR_Structure_Analyze(S,L); x=pinv(A)*b;
figure(3); RR_Structure_Plot(S,L,x); error_norm=norm(A*x-b), view(30,20), pause

%% 3-D  PART B : FRAME model 
disp('3-D bicycle frame -- PART B -- FRAME model')
S.C=[ 1  1  1  0  0  0  0;  
      0  0  1  1  0  0  0;  
      0  0  1  0  1  0  0;  
      0  1  0  1  0  0  0; 
      0  1  0  0  1  0  0;  
      1  0  0  0  0  1  0; 
      1  0  0  0  0  0  1];
[A,b,S,L]=RR_Structure_Analyze(S,L); x=pinv(A)*b;
figure(4); RR_Structure_Plot(S,L,x); error_norm=norm(A*x-b), view(30,20)
