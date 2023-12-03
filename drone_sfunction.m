function problem4b_sfunction(block)
%MPC S Function for problem4b hw4

%   Copyright 2003-2018 The MathWorks, Inc.

%%
%% The setup method is used to set up the basic attributes of the
%% S-function such as ports, parameters, etc. Do not add any other
%% calls to the main body of the function.
%%
setup(block);

%endfunction

%% Function: setup ===================================================
%% Abstract:
%%   Set up the basic characteristics of the S-function block such as:
%%   - Input ports
%%   - Output ports
%%   - Dialog parameters
%%   - Options
%%
%%   Required         : Yes
%%   C MEX counterpart: mdlInitializeSizes
%%
function setup(block)

% Register number of ports
block.NumInputPorts  = 12;
block.NumOutputPorts = 4;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% % Override input port properties
% block.InputPort(1).Dimensions        = 1;
% block.InputPort(1).DatatypeID  = 0;  % double
% block.InputPort(1).Complexity  = 'Real';
% block.InputPort(1).DirectFeedthrough = true;
% 
% % Override output port properties
% block.OutputPort(1).Dimensions       = 1;
% block.OutputPort(1).DatatypeID  = 0; % double
% block.OutputPort(1).Complexity  = 'Real';

% Register parameters
block.NumDialogPrms     = 0; %What happens when we double click the block

% Register sample times
%  [0 offset]            : Continuous sample time
%  [positive_num offset] : Discrete sample time
%
%  [-1, 0]               : Inherited sample time
%  [-2, 0]               : Variable sample time
block.SampleTimes = [-1 0]; 

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'CustomSimState',  < Has GetSimState and SetSimState methods
%    'DisallowSimState' < Error out when saving or restoring the model sim state
block.SimStateCompliance = 'DefaultSimState';

%% -----------------------------------------------------------------
%% The MATLAB S-function uses an internal registry for all
%% block methods. You should register all relevant methods
%% (optional and required) as illustrated below. You may choose
%% any suitable name for the methods and implement these methods
%% as local functions within the same file. See comments
%% provided for each function for more information.
%% -----------------------------------------------------------------
% block.RegBlockMethod('InitializeConditions', @InitializeConditions);
block.RegBlockMethod('Outputs', @Outputs);     % Required
block.RegBlockMethod('Terminate', @Terminate); % Required

%end setup
% 
% function InitializeConditions(block)
%     i = 1;
%     load("waypoints_file.mat");
%     waypoints = waypoint_gen(waypoints_checkpoint2);
%     current_waypoint_targ = waypoints(i)';

%%
%% Outputs:
%%   Functionality    : Called to generate block outputs in
%%                      simulation step
%%   Required         : Yes
%%   C MEX counterpart: mdlOutputs
%%
function Outputs(block)
    %   Extract state vector from input data
    currentX = block.InputPort(1).Data;
    currentY = block.InputPort(2).Data;
    currentZ = block.InputPort(3).Data;
    currentU = block.InputPort(4).Data;
    currentV = block.InputPort(5).Data;
    currentW = block.InputPort(6).Data;
    currentPhi = block.InputPort(7).Data;
    currentTheta = block.InputPort(8).Data;
    currentPsi = block.InputPort(9).Data;
    currentP = block.InputPort(10).Data;
    currentQ = block.InputPort(11).Data;
    currentR = block.InputPort(12).Data;
    current_pos = [currentX,currentY,currentZ,currentU,currentV,currentW,currentPhi,currentTheta,currentPsi,currentP,currentQ,currentR,]';
    
    dist_pos = [currentX,currentY,currentZ,currentTheta]';
    reach_cond_thresh = 0.4;
    if norm(current_waypoint_targ - dist_pos)< reach_cond_thresh
        if i < size(waypoints,1)
            i = i +1;
        end
    end
    
    current_waypoint_targ = waypoints(i,:)';


    
    
    %   Define the horizon length and time step
    N = 30;
    T_MPC = 0.5;
    
    %   Specify an initial guess (control input = angular rate)
    u0 = 0*ones(N,4);
    
    options = optimoptions('fmincon','Display','iter','Algorithm','sqp', 'MaxFunctionEvaluations',1200000);   %   Last entry sets the algorithm to SQP
    
    u_opt = fmincon(@(u)problem4b_objective(u,current_pos,N,T_MPC,current_waypoint_targ),u0,[],[],[],[],[],[],@(u)problem4b_constraints(u,current_pos,N,T_MPC),options);
    disp(size(u_opt))
    %   MPC output is the first step of the optimized control trajectory
    block.OutputPort(1).Data = u_opt(1,1);
    block.OutputPort(2).Data = u_opt(1,2);
    block.OutputPort(3).Data = u_opt(1,3);
    block.OutputPort(4).Data = u_opt(1,4);

%end Outputs

%%
%% Terminate:
%%   Functionality    : Called at the end of simulation for cleanup
%%   Required         : Yes
%%   C MEX counterpart: mdlTerminate
%%
function Terminate(block)

%end Terminate

