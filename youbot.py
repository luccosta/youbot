from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import math

print("Trying to connect")
client = RemoteAPIClient()
sim = client.require('sim')
print("Connected!")

def arm_configuration_callback(config, vel, accel, handles):
    for i, joint_value in enumerate(config):
        if sim.isDynamicallyEnabled(handles[i]):
            sim.setJointTargetPosition(handles[i], joint_value)
        else:
            sim.setJointPosition(handles[i], joint_value)


class YouBot:
    def __init__(self):
        self.wheels = {}
        self.wheels["front_left"] = sim.getObject('./rollingJoint_fl')
        self.wheels["rear_left"] = sim.getObject('./rollingJoint_rl')
        self.wheels["rear_right"] = sim.getObject('./rollingJoint_rr')
        self.wheels["front_right"] = sim.getObject('./rollingJoint_fr')

        if (self.wheels["front_left"] == -1): # TODO: completar o teste
            print("error")


        self.armJoints = []
        for i in range(5):
            self.armJoints.append(sim.getObject('./youBotArmJoint' + str(i)))

        self.gripperJoints = []
        for i in range(1, 3):
            self.gripperJoints.append(sim.getObject('./youBotGripperJoint' + str(i)))

        for handler in list(self.wheels.values()) + self.armJoints + self.gripperJoints:
            if handler == -1:
                print("handler failed")


    def move(self, forward_vel, left_vel, rot_vel):
        sim.setJointTargetVelocity(self.wheels["front_left"], - forward_vel - left_vel - rot_vel)
        sim.setJointTargetVelocity(self.wheels["rear_left"], - forward_vel + left_vel - rot_vel)
        sim.setJointTargetVelocity(self.wheels["rear_right"], - forward_vel - left_vel + rot_vel)
        sim.setJointTargetVelocity(self.wheels["front_right"], - forward_vel + left_vel + rot_vel)


    def arm_configuration(self, joint_values, closed_gripper=False):

        vel=180
        accel=40
        jerk=80
       
    
        maxVel=[vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180]
        maxAccel=[accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180]
        maxJerk=[jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180]


        current_config=[]
        for joint in self.armJoints:
            current_config.append(sim.getJointPosition(joint))
        sim.moveToConfig(-1, current_config, [0]*5, [0]*5, maxVel,maxAccel,maxJerk,joint_values, [0]*5, arm_configuration_callback, self.armJoints)

        if closed_gripper:
            sim.setJointTargetVelocity(self.gripperJoints[1], 1.0)

    
    """
    A junta 1 configura o deslocamento da garra inteira para a direcao do primeiro dedo e vai de 0 a 0.025. Default: 0.025.
    A junta 2 configura a velocidade somente do segundo dedo, com a posicao indo de -0.05 a 0.0 e a velocidade negativa abrir a garra. Default: -0.05.
    """
    def gripper_configuration(self, gripper_values):
        if sim.isDynamicallyEnabled(self.gripperJoints[0]):
            sim.setJointTargetPosition(self.gripperJoints[0], gripper_values[0])
        else:
            sim.setJointPosition(self.gripperJoints[0], gripper_values[0])

        sim.setJointTargetVelocity(self.gripperJoints[1], gripper_values[1])

def step_secs(secs, func):
    current = sim.getSimulationTime()
    while (t := sim.getSimulationTime()) - current < secs:
        print(f'Simulation time: {t:.2f} [s]')
        func()
        sim.step()

def main():
    sim.loadScene("/home/lucas/master/automacao_inteligente/youbot.ttt")

    robot = YouBot()
    sim.setStepping(True)

    sim.startSimulation()

    func = lambda: robot.move(0.5, 0, 0)
    step_secs(5, func)

    func = lambda: robot.move(0, 0.5, 0)
    step_secs(5, func)

    func = lambda: robot.move(0, 0, 0.5)
    step_secs(5, func)

    func = lambda: robot.arm_configuration([0.0, 0.65, 0.915, 1.27, 0.0])
    step_secs(5, func)

    func = lambda: robot.gripper_configuration([0.025, 1.0])
    step_secs(3, func)

    func = lambda: robot.arm_configuration([0.0, 0.5, 0.915, 1.27, 0.0])
    step_secs(5, func)

    func = lambda: robot.arm_configuration([0.87, 0.61, 0.915, 1.27, 0.0])
    step_secs(2, func)

    func = lambda: robot.gripper_configuration([0.025, -0.04])
    step_secs(2, func)

    sim.stopSimulation()

if __name__ == "__main__":
    main()