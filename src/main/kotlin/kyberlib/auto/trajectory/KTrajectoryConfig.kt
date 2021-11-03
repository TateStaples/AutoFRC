package kyberlib.auto.trajectory

import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint
import frc.team6502.robot.SlamValues
import kotlinx.serialization.Serializable
import kotlinx.serialization.decodeFromString
import kotlinx.serialization.encodeToString
import kotlinx.serialization.json.Json
import kyberlib.math.units.extensions.LinearVelocity
import kyberlib.math.units.extensions.feetPerSecond
import kyberlib.math.units.extensions.metersPerSecond
import java.io.File

class KTrajectoryConfig(maxVelocity: LinearVelocity, maxAcceleration: LinearVelocity,
                             constraints: List<TrajectoryConstraint> = listOf(),
                             initialVelocity: LinearVelocity = 0.feetPerSecond, finalVelocity: LinearVelocity = 0.feetPerSecond,
                             reversed: Boolean = false
                        ) : TrajectoryConfig(maxVelocity.metersPerSecond, maxAcceleration.metersPerSecond) {
    private val data = TrajectoryConfigData(maxVelocity.metersPerSecond, maxAcceleration.metersPerSecond, initialVelocity.metersPerSecond, finalVelocity.metersPerSecond, reversed)

    init {
        isReversed = reversed
        startVelocity = initialVelocity.metersPerSecond
        endVelocity = finalVelocity.metersPerSecond
        addConstraints(constraints)
    }

    fun save(file: File) {
        file.writeText(Json.encodeToString(data))
    }

    companion object {
        fun load(file: File): KTrajectoryConfig {
            val data = Json.decodeFromString<TrajectoryConfigData>(file.readText())
            return KTrajectoryConfig(data.maxVelocity.metersPerSecond, data.maxAcceleration.metersPerSecond, listOf(), data.initialVelocity.metersPerSecond, data.finalVelocity.metersPerSecond, data.reversed)
        }
    }
}

@Serializable
internal data class TrajectoryConfigData(val maxVelocity: Double, val maxAcceleration: Double,
                                         val initialVelocity: Double, val finalVelocity: Double,
                                         val reversed: Boolean)
