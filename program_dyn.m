% Before running this script, open the scenario in VREP, e.g
% Do not run simulation!

% Try to connect to simulator
vehicle = RobDyn_robot();
%initialize robot class

timestep = vehicle.get_simulation_timestep();
%get time step value (normally dt=50ms)

[Rrobot,psi_obs] = vehicle.get_RobotCharacteristics();
%Rrobot - robot radius (in cm)
%psi_obs - Angle value for each sensor i (i = 1, . . . , 11) relative to the
%frontal direction (in rad)

%------------------------------------------------------------------------------
%for future
%param = dyn_obstar_parameters(timestep,Rrobot,psi_obs);
%param = dyn_tar_parameters(timestep,Rrobot,psi_obs);
%param = dyn_obs_parameters(timestep,Rrobot,psi_obs);
%------------------------------------------------------------------------------

% Set initial velocity vrobot = 0 (linear velocity) and wrobot = 0 (angular velocity)
wrobot = 0; %rad/s
vrobot = 0; %cm/s

ntarget=1; %initialize first target 


%Parametros de controlo
lambda_alvo=1.5;
lambda_velocidade=-0.85;
Vdes=30;

while ntarget<=vehicle.TARGET_Number % until robot goes to target 2
   
   %% Robot interface
   % set and get information to/from vrep
   % avoid do processing in between ensure_all_data and trigger_simulation
   vehicle.ensure_all_data();
   
   % set robot angular velocity (rad/s) and robot linear velocity (cm/s)
   % get also pose. it can be used vehicle.get_vehicle_pose() instead
   [xrobot, yrobot, phirobot] = vehicle.set_velocity(wrobot, vrobot);   
   % robot position (xrobot,yrobot) in cm
   % phirobot - rad
   % wrobot - rad/s
   % vrobot - cm/s
   
   %Get distance from obstacles (11 infrared obstacles - distance between
   %0-80cm)
   [dc] = vehicle.get_DistanceSensorAquisition();  %distance in cm
   
   %Get target position for target 1 or 2 (t)
   [XTARGET,YTARGET] = vehicle.get_TargetPosition(ntarget); 
   %position (XTARGET,YTARGET) in cm
  
   %get simulation time
   sim_time = vehicle.get_simulation_time();
   
   %trigger simulation step
   vehicle.trigger_simulation();
  
   %% Processing step
   % compute new vehicle velocity...
   % the simulation timestep is stored in timestep (value is not changed
   % while simulation is running)
   % the simulation time is stored in sim_time.
   
   % --- YOUR CODE --- %
   vrobot=60;
   g=lambda_velocidade*(vrobot-Vdes);
   vrobot=vrobot+g;
   wrobot=-lambda_alvo * sin(phirobot - atan2(YTARGET-yrobot,XTARGET-xrobot));
   
   
   %vrobot - linear velocity
   %wrobot - angular velocity
   
   % phirobot - rad
   % wrobot - rad/s
   % vrobot - cm/s
   
   % dc - vector com distancias para cada sensor
   
   % XTARGET - x position of target
   % YTARGET - y position of target
   
   %-------------------------------------------------------------------------------
   %For future
   %[wrobot,vrobot,param,ntarget] = dyn_obstar_phi(param, dc, xrobot, yrobot, phirobot, XTARGET, YTARGET,ntarget);
   %[wrobot,vrobot,param,ntarget] = dyn_tar_phi(param, dc, xrobot, yrobot, phirobot, XTARGET, YTARGET,ntarget);
   %[wrobot,vrobot,param,ntarget] = dyn_obs_phi(param, dc, xrobot, yrobot, phirobot, XTARGET, YTARGET,ntarget);
   %-------------------------------------------------------------------------------

end

vehicle.terminate();
    
    