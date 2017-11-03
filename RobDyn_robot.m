classdef RobDyn_robot < handle
    %RobDyn_ROBOT Interface to a simulated robot in VREP simulator
    %   Detailed explanation goes here
    
    properties (Access = private)
        vrep
        clientID
        
        RobotHandle
        steering_motor_handle
        TargetHandle
        sensorHandle
        LeftMotorHandle
        RightMotorHandle
  
        world_ref_handle
        
    end
    
    properties
        name
        TARGET_Number
        Rrobot
        Obst
        NsensorsIRobs
        Dwheels
        RadiusWheels
    end
    
    methods
        function obj = RobDyn_robot()
            
            obj.TARGET_Number=2;
            
            obj.Rrobot = 45.0/2;
            
            obj.Dwheels = 0.35;    %m
            
            obj.RadiusWheels = 0.10/2;    %m
            
            obj.NsensorsIRobs = 11; 
            
            obj.vrep = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
            obj.vrep.simxFinish(-1); % just in case, close all opened connections

            obj.clientID = obj.vrep.simxStart('127.0.0.1', 19997, true, true, 5000, 5);
                       
            if (obj.clientID > -1)
                fprintf('\n');
                disp('INFO: Connected successfully to V-REP!');
                fprintf('\n');
            else
                clear obj;
                msg = 'ERROR: Failed connecting to remote API server. Ensure that the simulation is running in V-REP.\n';
                error(msg)
            end
            
            %% Synchronous mode
            obj.vrep.simxSynchronous(obj.clientID, true); % Enable the synchronous mode 
            obj.vrep.simxStartSimulation(obj.clientID, obj.vrep.simx_opmode_oneshot);
            
            obj.vrep.simxSynchronousTrigger(obj.clientID); % Trigger next simulation step (Blocking function call)
            
           % The first simulation step is now being executed
           %ensure it is finished so we can access signals
            obj.vrep.simxGetPingTime(obj.clientID);
            
            %% Get objects handles
            obj.world_ref_handle = -1;
            
            obj_name = 'RobotDyn';
            [res, obj.RobotHandle] = obj.vrep.simxGetObjectHandle(obj.clientID, obj_name, obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed getting robot handle ';
                error(msg);
            end
            
            [res,~] = obj.vrep.simxGetObjectPosition(obj.clientID,obj.RobotHandle,-1,obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed getting robot position information \n';
                error(msg);
            end

            [res,~] = obj.vrep.simxGetObjectOrientation(obj.clientID,obj.RobotHandle,-1,obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed getting robot orientation information \n';
                error(msg);
            end
            
            % target handle

            for t=1:obj.TARGET_Number
                targetName{t} = ['Target',num2str(t)];
                [res,obj.TargetHandle{t}]=obj.vrep.simxGetObjectHandle(obj.clientID,targetName{t},obj.vrep.simx_opmode_blocking);

                if (res ~= obj.vrep.simx_return_ok)
                    msg = 'ERROR: Failed getting target handle ';
                    error(msg);
                end
                
                [res,~] = obj.vrep.simxGetObjectPosition(obj.clientID,obj.TargetHandle{t},-1,obj.vrep.simx_opmode_streaming);
                if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                    msg = 'ERROR: Failed getting target information \n';
                    error(msg);
                end
            end


            %sensor handles
            m=1;

            for j=1:2:21

                sensorName{j} = ['RobotDyn_s',num2str(j)];

                [res,obj.sensorHandle{m}]=obj.vrep.simxGetObjectHandle(obj.clientID,sensorName{j},obj.vrep.simx_opmode_blocking);

                if (res ~= obj.vrep.simx_return_ok)
                    msg = 'ERROR: Failed getting sensor handle ';
                    error(msg);
                end
                [res,~,~,~,~]=obj.vrep.simxReadProximitySensor(obj.clientID, obj.sensorHandle{m},obj.vrep.simx_opmode_streaming);

                if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                    msg = 'ERROR: Failed getting sensor data \n';
                    error(msg);
                end

                m=m+1;
            end

            % robot handle for motors
            [res,obj.LeftMotorHandle]=obj.vrep.simxGetObjectHandle(obj.clientID,'RobotDyn_lMotor',obj.vrep.simx_opmode_blocking);

            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed getting Left Motor handle ';
                error(msg);
            end

            [res,obj.RightMotorHandle]=obj.vrep.simxGetObjectHandle(obj.clientID,'RobotDyn_rMotor',obj.vrep.simx_opmode_blocking);

            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed getting Right Motor handle ';
                error(msg);
            end
            
  

        
            
            %% Setup streaming of data
            % simulation time
            [res, ~] = obj.vrep.simxGetFloatSignal(obj.clientID,'SimulationTime', obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to setup data streaming for simulation time';
                error(msg);
            end
  
        end
        
        %call before setting velocity and getting data
        function ensure_all_data(obj)
            obj.vrep.simxGetPingTime(obj.clientID);
        end
        
        % call just after all data have been collected.
        function trigger_simulation(obj)
            obj.vrep.simxSynchronousTrigger(obj.clientID);
        end
        
        function [Rrobot,Obst] = get_RobotCharacteristics(obj)
            Rrobot = obj.Rrobot;                
            distance_between_sensors = 22.5*pi/180;   %rad

            for i=1:obj.NsensorsIRobs
                Obst(i) = ((-fix(obj.NsensorsIRobs/2.0) + i - 1.0) * distance_between_sensors);
            end
        end
        
        function  [x, y, phi] = set_velocity(obj, wrobot, vrobot) %wrobot - rad/s; vrobot - cm/s
            %% Set vehicle velocity    
            dv = wrobot*obj.Dwheels/2.0;  	%m/s , Wrobot = (vl-vr)/Dist_between_the_wheels*/
            vLeft = vrobot*0.01-dv;      	%m/s*/
            vRight = vrobot*0.01+dv;        %m/s*/
            
            wLeft = vLeft/obj.RadiusWheels;
            res = obj.vrep.simxSetJointTargetVelocity(obj.clientID, obj.LeftMotorHandle, wLeft, obj.vrep.simx_opmode_oneshot );
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed sending vLeft data! \n';
                error(msg);
            end
            
            wRight = vRight/obj.RadiusWheels;
            res = obj.vrep.simxSetJointTargetVelocity(obj.clientID, obj.RightMotorHandle, wRight, obj.vrep.simx_opmode_oneshot);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed sending angle data! \n';
                error(msg);
            end
            
            % Get pose
            [x, y, phi] = get_vehicle_pose(obj);
            
        end
        
        function [x, y, phi] = get_vehicle_pose(obj)
            % Get pose
            % x and y in cm
            % phi in rad
            [res, robot_pos] = obj.vrep.simxGetObjectPosition(obj.clientID, obj.RobotHandle, -1, obj.vrep.simx_opmode_oneshot);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'Failed getting robot position!';
                error(msg);
            end
            
            [res, robot_ori] = obj.vrep.simxGetObjectOrientation(obj.clientID, obj.RobotHandle, -1, obj.vrep.simx_opmode_oneshot);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'Failed getting robot orientation!';
                error(msg);
            end
            
            x = robot_pos(1)*100;   %cm
            y = robot_pos(2)*100;   %cm
            phi = robot_ori(3);     %rad/s
        end
        
         function [x, y, phi] = get_vehicle_pose2pi(obj)
             [x, y, phi] = get_vehicle_pose(obj);
             phi = rem(phi + 2*pi, 2*pi);
         end
         
         function [XTARGET,YTARGET] = get_TargetPosition( obj, t)
            %UNTITLED2 Summary of this function goes here
            %   Detailed explanation goes here
            %XTARGET and YTARGET in cm

            [res,tposition] = obj.vrep.simxGetObjectPosition(obj.clientID,obj.TargetHandle{t},-1,obj.vrep.simx_opmode_buffer);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed getting target position! \n';
                error(msg);
            end

            XTARGET=tposition(1)*100;      %cm
            YTARGET=tposition(2)*100;      %cm
         end
            
         function [dc] = get_DistanceSensorAquisition( obj )
            %UNTITLED Summary of this function goes here
            %   Detailed explanation goes here
            %dc in cm

            dc = zeros(1,obj.NsensorsIRobs);

            for s=1:obj.NsensorsIRobs,
                [res,detectionState,detectedPoint,~,~]=obj.vrep.simxReadProximitySensor(obj.clientID, obj.sensorHandle{s},obj.vrep.simx_opmode_buffer);

                if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                    msg = 'ERROR: Failed getting sensor data \n';
                    error(msg);
                end

                if detectionState==0
                    dc(s)=80;       %cm
                else
                    dc(s)=detectedPoint(3)*100; %cm
                end
            end

        end
        
        function sim_time = get_simulation_time(obj)
            [res, sim_time] = obj.vrep.simxGetFloatSignal(obj.clientID, 'SimulationTime', obj.vrep.simx_opmode_buffer);
            if (res ~= obj.vrep.simx_return_ok )
                msg = 'ERROR: Failed getting simulation time! \n';
                error(msg);
            end
        end
        
        function sim_timestep = get_simulation_timestep(obj)
            [res, sim_timestep] = obj.vrep.simxGetFloatSignal(obj.clientID, 'SimulationTimeStep', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok )
                msg = 'ERROR: Failed getting simulation time step! \n';
                error(msg);
            end
        end
        
        function terminate(obj)
           
            res = obj.vrep.simxStopSimulation(obj.clientID, obj.vrep.simx_opmode_oneshot);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed stopping simulation! \n';
                error(msg);
            end
            obj.vrep.simxGetPingTime(obj.clientID);

            % Now close the connection to V-REP:    
            obj.vrep.simxFinish(obj.clientID);

            obj.vrep.delete(); % call the destructor!
        end
    end
    
end

