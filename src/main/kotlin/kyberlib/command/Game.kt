package kyberlib.command

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.RobotController


object Game {
    private val DS = DriverStation.getInstance()

    val real
        get() = RobotBase.isReal()
    val sim
        get() = RobotBase.isSimulation()

    val enabled
        get() = DS.isEnabled
    val disabled
        get() = DS.isDisabled
    val AUTO
        get() = DS.isAutonomousEnabled
    val TELEOP
        get() = DS.isOperatorControlEnabled
    val TEST
        get() = DS.isTest

    val brownedOut
        get() = RobotController.isBrownedOut()
    val CAN
        get() = RobotController.getCANStatus()
    val batteryVoltage
        get() = RobotController.getBatteryVoltage()

    val time = RobotController.getFPGATime()
}