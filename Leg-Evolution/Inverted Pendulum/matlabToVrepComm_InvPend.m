% This small example illustrates how to use the remote API
% synchronous mode. The synchronous mode needs to be
% pre-enabled on the server side. You would do this by
% starting the server (e.g. in a child script) with:
%
% simExtRemoteApiStart(19999,1300,false,true)
%
% But in this example we try to connect on port
% 19997 where there should be a continuous remote API
% server service already running and pre-enabled for
% synchronous mode.
%
% IMPORTANT: for each successful call to simxStart, there
% should be a corresponding call to simxFinish at the end!

close all
clear all
clc

%disp('Program started');
% vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);

if (clientID>-1)
    disp('Connected to remote API server');
    
    
    %%%% Reset Sim %%%%
    
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);
    
    [~, ankleJointHandle]=vrep.simxGetObjectHandle(clientID,'Revolute_joint',vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,ankleJointHandle,70*pi/180,vrep.simx_opmode_blocking);
    %     pause(1)
    
    %%%%%%%%%%%%%%%%%%%
    
    % enable the synchronous mode on the client:
    vrep.simxSynchronous(clientID,true);
    
    % start the simulation:
    vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);
    
%     [~, ankleJointHandle]=vrep.simxGetObjectHandle(clientID,'Revolute_joint',vrep.simx_opmode_blocking);
    
    % Start the Streaming (To allow V-REP to put values into the buffer
    
    [~,position]=vrep.simxGetJointPosition(clientID,ankleJointHandle,vrep.simx_opmode_streaming);
    [~, jointTorque]=vrep.simxGetJointForce(clientID,ankleJointHandle,vrep.simx_opmode_streaming);
    
    count=0; % Variable for cycle counting
    
    f=1;
    n=0; % step counting (for graph-plotting)
    dt=50; %ms
    
    
    % Now step a few times:
    while count <11
        vrep.simxSynchronousTrigger(clientID);
        
        n=n+1;
        
        %%% The values for a certain step are only put in the buffer after a trigger to start
        %%% the next step
        
        [~,position]=vrep.simxGetJointPosition(clientID,ankleJointHandle,vrep.simx_opmode_buffer); % Position for the previous step
        [~, jointTorque]=vrep.simxGetJointForce(clientID,ankleJointHandle,vrep.simx_opmode_buffer); % Torque for the previous step
        
        vecIter(n)=n;
        vecPos(n)=position;
        vecTorq(n)=jointTorque;
        
        
        pause(.01) % To allow real-time plotting
        
        %%% Graph-Plotting
        
        figure(1)
        subplot(1,2,1)
        plot( (vecIter-vecIter(1) )*dt,vecPos)
        xlabel('Step Time (ms)')
        ylabel('Joint Angle (rad)')
        title('Time-Position')
        
        subplot(1,2,2)
        plot( (vecIter-vecIter(1) )*dt,vecTorq)
        xlabel('Step Time (ms)')
        ylabel('Joint Torque (N*m)')
        title('Time-Torque')
        
        
        if (180*position/pi < f*70+1) && (180*position/pi > f*70-1)
            f=-f;
            count=count+1;
            
            [returnCode]=vrep.simxSetJointTargetPosition(clientID,ankleJointHandle,f*70*pi/180,vrep.simx_opmode_blocking);
        end
        %         pause;
    end
    
    % stop the simulation:
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);
    
    % Now close the connection to V-REP:
    vrep.simxFinish(clientID);
else
    disp('Failed connecting to remote API server');
end
vrep.delete(); % call the destructor!

% figure;
% plot(m,p)

%disp('Program ended');

