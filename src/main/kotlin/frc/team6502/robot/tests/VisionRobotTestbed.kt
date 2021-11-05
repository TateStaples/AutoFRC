package frc.team6502.robot.tests

import edu.wpi.first.wpilibj.RobotBase
import frc.team6502.robot.auto.cv.Photon
import frc.team6502.robot.auto.cv.SlamBridge
import kyberlib.command.KRobot

class VisionRobotTestbed : KRobot() {
    init {
        if (RobotBase.isReal())
            Photon()
        else
            SlamBridge()
    }
    override fun simulationInit() {
        // this should be handled by slamBridge
    }

    override fun simulationPeriodic() {
        // this should be handled by slamBridge
    }
}













