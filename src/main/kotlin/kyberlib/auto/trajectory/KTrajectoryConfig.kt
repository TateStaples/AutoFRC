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

/**
 * Wrapper of trajectory configs that takes in KUnits and allows for saving/loading
 */
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

    /**
     * Save this config to class. Leaves out constraints cause idk how to add that
     */
    fun save(file: File) {
        file.writeText(Json.encodeToString(data))
    }

    companion object {
        /**
         * Loads config from JSON file.
         * JSON should be written from save function
         */
        fun load(file: File): KTrajectoryConfig {
            val data = Json.decodeFromString<TrajectoryConfigData>(file.readText())
            return KTrajectoryConfig(data.maxVelocity.metersPerSecond, data.maxAcceleration.metersPerSecond, listOf(), data.initialVelocity.metersPerSecond, data.finalVelocity.metersPerSecond, data.reversed)
        }
    }
}

/**
 * Internal Config Data that allows for Serialization (writing to JSON)
 */
@Serializable
internal data class TrajectoryConfigData(val maxVelocity: Double, val maxAcceleration: Double,
                                         val initialVelocity: Double, val finalVelocity: Double,
                                         val reversed: Boolean)
