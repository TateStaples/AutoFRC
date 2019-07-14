package frc.team6502.robot

import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.TimedRobot

class Robot : TimedRobot() {

    override fun robotInit() {
        // report language as kotlin instead of assuming java because of JVM
        HAL.report(FRCNetComm.tResourceType.kResourceType_Language, 6)

        // initialize RobotMap
        RobotMap
    }

    override fun robotPeriodic() {

    }

    override fun disabledInit() {

    }

    override fun disabledPeriodic() {

    }

    override fun autonomousInit() {

    }

    override fun autonomousPeriodic() {

    }

    override fun teleopInit() {

    }

    override fun teleopPeriodic() {

    }

}