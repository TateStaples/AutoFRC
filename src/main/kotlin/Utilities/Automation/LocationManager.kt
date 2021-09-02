package Utilities.Automation

import edu.wpi.first.wpilibj.geometry.Pose2d
import Utilities.Automation.Map
import com.ctre.phoenix.sensors.PigeonIMU
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

/*
Sources:
- Sensor Fusion (https://www.youtube.com/watch?v=6qV3YjFppuc)
- Kalman Filters (https://www.youtube.com/watch?v=mwn8xhgNpFY)
 */
class LocationManager(initialLocation: Pose2d) {
    val pose
        get() = odometry.poseMeters
    val odometry = DifferentialDriveOdometry(initialLocation.rotation, initialLocation)
    val path = mutableListOf<Pose2d>()
    var certaintyField = Array(10) { DoubleArray(10) }


    fun updatePose(newPose: Pose2d) {
        TODO("update this with kalman filtering")
        val x = (pose.x * LocationManagerPreferences.odometryVariance + newPose.x * LocationManagerPreferences.cameraUpdateVariance) / (LocationManagerPreferences.cameraUpdateVariance + LocationManagerPreferences.odometryVariance)
        val y = (pose.y * LocationManagerPreferences.odometryVariance + newPose.y * LocationManagerPreferences.cameraUpdateVariance) / (LocationManagerPreferences.cameraUpdateVariance + LocationManagerPreferences.odometryVariance)
//        val theta = (pose.x * LocationManagerPreferences.odometryVariance + newPose.x * LocationManagerPreferences.cameraUpdateVariance) / (LocationManagerPreferences.cameraUpdateVariance + LocationManagerPreferences.odometryVariance)
        val ajustedPose = Pose2d(x, y, pose.rotation)
        odometry.resetPosition(ajustedPose, pose.rotation)
    }

    fun updateOdometry(angle: Rotation2d, leftDistance: Double, rightDistance: Double) {
        path.add(pose)
        odometry.update(angle, leftDistance, rightDistance)
    }

    fun display() {
        SmartDashboard.putNumber("x", pose.x)
        SmartDashboard.putNumber("y", pose.y)
        SmartDashboard.putNumber("theta", pose.rotation.degrees)
    }

}

object LocationManagerPreferences {
    // maybe have odometry variance change with some integral of time
    var cameraUpdateVariance = 0.0
    var odometryVariance = 0.0
}