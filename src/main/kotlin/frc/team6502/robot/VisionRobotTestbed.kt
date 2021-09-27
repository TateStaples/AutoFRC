package frc.team6502.robot

import frc.team6502.robot.auto.cv.Camera
import kyberlib.command.KRobot

class VisionRobotTestbed : KRobot() {
    init {
        Camera
    }

    override fun robotPeriodic() {
        println(Camera.currentImage.size().toString())
    }
}