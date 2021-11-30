package frc.team6502.robot

import frc.team6502.robot.tests.VisionRobotTestbed
import kyberlib.simulation.Simulation

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
            if(Simulation.real) Robot().initialize()
            else VisionRobotTestbed().initialize()
        }
    }
}

