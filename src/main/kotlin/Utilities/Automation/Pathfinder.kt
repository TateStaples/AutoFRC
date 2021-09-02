package Utilities.Automation

import edu.wpi.first.wpilibj.geometry.Pose2d
import frc.team6502.kyberlib.auto.KTrajectory

class Pathfinder(val location: LocationManager, val map: Map) {

    fun goTo(setpoint: Pose2d): List<KTrajectory> {
        return listOfNotNull<KTrajectory>()
    }
}