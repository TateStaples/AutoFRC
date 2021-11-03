package kyberlib.command

import edu.wpi.first.wpilibj.Sendable
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

/**
 * Inheritable class that grants multiple types of debugging
 */
abstract class Debuggable {
    /**
     * Debugs all the values into a group in the SmartDashboard
     */
    fun debugDashboard() {
        val map = debugValues()
        for (info in map) {
            val path = "$identifier/${info.key}"
            if (info.value is Boolean)
                SmartDashboard.putBoolean(path, info.value as Boolean)
            else if (info.value is Double)
                SmartDashboard.putNumber(path, info.value as Double)
            else if (info.value is Sendable)
                SmartDashboard.putData(info.key, info.value as Sendable)
            else
                SmartDashboard.putString(path, info.value.toString())
        }
    }

    /**
     * Prints out all the values in a neat way
     */
    fun debugPrint() {
        val map = debugValues()
        val stringBuilder = StringBuilder()
        stringBuilder.append("$identifier - ")
        for (info in map) {
            stringBuilder.append("${info.key}: ${info.value}, ")
        }
        println(stringBuilder.toString())
    }

    /**
     * The name of the group of values.
     * Defaults to the name of the calling class
     */
    protected open val identifier: String
        get() = javaClass.simpleName

    /**
     * Function that retrieves the values that need to be debugged
     * @return map of string (name) to value. Numbers, Booleans, and Sendables are displayed as such
     */
    abstract fun debugValues(): Map<String, Any?>
}