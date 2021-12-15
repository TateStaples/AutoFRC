package kyberlib.command

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Sendable
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team6502.robot.subsystems.Drivetrain

/**
 * Types of ways to print to the output (driverstation)
 * - Print = normal print statement. Will typically remain in console and white
 * - Warn = yellow font and shows in the small outputs
 * - Error = Bright red letters and will show as error
 */
enum class LogMode {  // todo: add types (ie important, default, all)
    PRINT, WARN, ERROR
}

enum class DebugLevel {
    LowPriority, NORMAL, HighPriority, MaxPriority
}
/**
 * Inheritable class that grants multiple types of debugging
 */
interface Debug {
    companion object {
        var debugging = true
        var loggingLevel = DebugLevel.NORMAL

        fun log(identifier:String, text: String, mode: LogMode = LogMode.PRINT, level: DebugLevel = DebugLevel.NORMAL, stacktrace: Boolean = false) {
            if (level < loggingLevel) return
            val output = "[$identifier] $text"
            when (mode) {
                LogMode.PRINT -> println(output)
                LogMode.WARN -> DriverStation.reportWarning(output, stacktrace)
                LogMode.ERROR -> DriverStation.reportError(output, stacktrace)
            }
        }
    }

    /**
     * Debugs all the values into a group in the SmartDashboard
     */
    fun debugDashboard(previousPath: String = "", id: String = identifier) {
        if (!debugging || loggingLevel < priority) return
        val map = debugValues()
        sendMapToDashboard(map, "$previousPath/$id")
    }

    private fun sendMapToDashboard(map: Map<String, Any?>, rootPath: String) {
        for (info in map) {
            val path = "$rootPath/${info.key}"
            when (info.value) {  // send each datum as their own type
                is Boolean -> SmartDashboard.putBoolean(path, info.value as Boolean)
                is Double -> SmartDashboard.putNumber(path, info.value as Double)
                is Debug -> sendMapToDashboard((info.value as Debug).debugValues(), path)
                is Map<*, *> -> sendMapToDashboard(info.value as Map<String, Any?>, path)
                is Sendable -> SmartDashboard.putData(path, info.value as Sendable)
                else -> SmartDashboard.putString(path, info.value.toString())
            }
        }
    }


    /**
     * Prints out all the values in a neat way
     * @param message the output that will be sent to console
     * @param logMode the way the message should appear
     * @see LogMode
     */
    fun log(message: String = debugString, logMode: LogMode = LogMode.PRINT) {
        Companion.log(identifier, message, logMode, priority, stacktrace = false)
    }

    private val debugString: String
        get() {
            val map = debugValues()
            val stringBuilder = StringBuilder()
            stringBuilder.append("$identifier - ")
            for (info in map) {
                val string = if (info.value is Debug) "(${(info.value as Debug).debugString})" else info.value.toString()
                stringBuilder.append("${info.key}: ${string}, ")
            }
            return stringBuilder.toString()
        }

    /**
     * The name of the group of values.
     * Defaults to the name of the calling class
     */
    var identifier: String
        get() = javaClass.simpleName
        set(value) = log("override me, don't set. Interfaces be wack", LogMode.ERROR)

    var priority: DebugLevel
        get() = DebugLevel.NORMAL
        set(value) = log("override me, don't set. Interfaces be wack", LogMode.ERROR)

    /**
     * Function that retrieves the values that need to be debugged
     * @return map of string (name) to value. Numbers, Booleans, and Sendables are displayed as such
     */
    fun debugValues(): Map<String, Any?>
}