% Initialize CoppeliaSim connection
disp('Trying to connect');
client = RemoteAPIClient();
sim = client.require('sim');
disp('Connected!');

% Define arm configuration callback
function arm_configuration_callback(config, vel, accel, handles)
    for i = 1:length(config)
        if sim.isDynamicallyEnabled(handles(i))
            sim.setJointTargetPosition(handles(i), config(i));
        else
            sim.setJointPosition(handles(i), config(i));
        end
    end
end

% Define YouBot class
classdef YouBot
    properties
        wheels
        armJoints
        gripperJoints
    end
    methods
        function obj = YouBot()
            obj.wheels = struct();
            obj.wheels.front_left = sim.getObject('./rollingJoint_fl');
            obj.wheels.rear_left = sim.getObject('./rollingJoint_rl');
            obj.wheels.rear_right = sim.getObject('./rollingJoint_rr');
            obj.wheels.front_right = sim.getObject('./rollingJoint_fr');
            
            % Check for errors
            if obj.wheels.front_left == -1
                disp('error');
            end
            
            obj.armJoints = zeros(1, 5);
            obj.gripperJoints = zeros(1, 2);
            for i = 1:5
                obj.armJoints(i) = sim.getObject(['./youBotArmJoint' num2str(i-1)]);
            end
            for i = 1:2
                obj.gripperJoints(i) = sim.getObject(['./youBotGripperJoint' num2str(i)]);
            end
            
            % Check for handler failures
            for handler = [values(obj.wheels), obj.armJoints, obj.gripperJoints]
                if handler == -1
                    disp('handler failed');
                end
            end
        end
        
        function move(obj, forward_vel, left_vel, rot_vel)
            sim.setJointTargetVelocity(obj.wheels.front_left, - forward_vel - left_vel - rot_vel);
            sim.setJointTargetVelocity(obj.wheels.rear_left, - forward_vel + left_vel - rot_vel);
            sim.setJointTargetVelocity(obj.wheels.rear_right, - forward_vel - left_vel + rot_vel);
            sim.setJointTargetVelocity(obj.wheels.front_right, - forward_vel + left_vel + rot_vel);
        end
        
        function arm_configuration(obj, joint_values, closed_gripper)
            vel = 180;
            accel = 40;
            jerk = 80;
            maxVel = vel * ones(1, 6) * pi / 180;
            maxAccel = accel * ones(1, 6) * pi / 180;
            maxJerk = jerk * ones(1, 6) * pi / 180;
            
            current_config = zeros(1, 5);
            for i = 1:5
                current_config(i) = sim.getJointPosition(obj.armJoints(i));
            end
            
            sim.moveToConfig(-1, current_config, zeros(1, 5), zeros(1, 5), maxVel, maxAccel, maxJerk, joint_values, zeros(1, 5), @arm_configuration_callback, obj.armJoints);
            
            if closed_gripper
                sim.setJointTargetVelocity(obj.gripperJoints(2), 1.0);
            end
        end
        
        function gripper_configuration(obj, gripper_values)
            if sim.isDynamicallyEnabled(obj.gripperJoints(1))
                sim.setJointTargetPosition(obj.gripperJoints(1), gripper_values(1));
            else
                sim.setJointPosition(obj.gripperJoints(1), gripper_values(1));
            end
            sim.setJointTargetVelocity(obj.gripperJoints(2), gripper_values(2));
        end
    end
end

% Define step function
function step_secs(secs, func)
    current = sim.getSimulationTime();
    while (sim.getSimulationTime() - current) < secs
        t = sim.getSimulationTime();
        disp(['Simulation time: ' num2str(t, '%.2f') ' [s]']);
        func();
        sim.step();
    end
end

% Main function
function main()
    sim.loadScene('/home/lucas/master/automacao_inteligente/youbot.ttt');
    
    robot = YouBot();
    sim.setStepping(true);
    sim.startSimulation();
    
    func = @() robot.move(0.5, 0, 0);
    step_secs(5, func);
    
    func = @() robot.move(0, 0.5, 0);
    step_secs(5, func);
    
    func = @() robot.move(0, 0, 0.5);
    step_secs(5, func);
    
    func = @() robot.arm_configuration([0.0, 0.65, 0.915, 1.27, 0.0]);
    step_secs(5, func);
    
    func = @() robot.gripper_configuration([0.025, 1.0]);
    step_secs(3, func);
    
    func = @() robot.arm_configuration([0.0, 0.5, 0.915, 1.27, 0.0]);
    step_secs(5, func);
    
    func = @() robot.arm_configuration([0.87, 0.61, 0.915, 1.27, 0.0]);
    step_secs(2, func);
    
    func = @() robot.gripper_configuration([0.025, -0.04]);
    step_secs(2, func);
    
    sim.stopSimulation();
end

% Execute main function
main();
