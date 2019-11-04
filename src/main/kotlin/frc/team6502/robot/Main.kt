package frc.team6502.robot

import edu.wpi.first.wpilibj.RobotBase

/**
 * Entry point. DO NOT TOUCH THIS FILE.
 */
class Main() {
    companion object {
        /**
         * Main initialization function. Do not perform any initialization here.
         */
        @JvmStatic
        fun main(args: Array<String>) {
            RobotBase.startRobot(::Robot)
        }
    }
}

