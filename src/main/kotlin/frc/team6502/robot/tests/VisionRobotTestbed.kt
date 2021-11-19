package frc.team6502.robot.tests

import frc.team6502.robot.auto.cv.SlamBridge
import kyberlib.command.KRobot

class VisionRobotTestbed : KRobot() {
    override fun simulationInit() {
        SlamBridge()
    }
}
