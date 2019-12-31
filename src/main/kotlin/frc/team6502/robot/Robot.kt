package frc.team6502.robot

import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.hal.sim.DriverStationSim
import edu.wpi.first.wpilibj.TimedRobot

/**
 * Main robot class.
 */
class Robot : TimedRobot() {

    override fun robotInit() {
        // report language as kotlin instead of assuming java because of JVM
        HAL.report(FRCNetComm.tResourceType.kResourceType_Language, FRCNetComm.tInstances.kLanguage_Kotlin)

        // initialize RobotContainer and by extension subsystems
        RobotContainer
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
