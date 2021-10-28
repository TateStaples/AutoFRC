package kyberlib.command

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard


abstract class Debuggable {
    fun debugDashboard() {
        val map = debugValues()
        for (info in map) {
            val path = "$identifier/${info.key}"
            if (info.value is Boolean)
                SmartDashboard.putBoolean(path, info.value as Boolean)
            else if (info.value is Double)
                SmartDashboard.putNumber(path, info.value as Double)
            else
                SmartDashboard.putString(path, info.value.toString())
        }
    }

    fun debugPrint() {
        val map = debugValues()
        val stringBuilder = StringBuilder()
        stringBuilder.append("$identifier - ")
        for (info in map) {
            stringBuilder.append("${info.key}: ${info.value}, ")
        }
        println(stringBuilder.toString())
    }

    protected open val identifier: String
        get() = javaClass.simpleName
    abstract fun debugValues(): Map<String, Any?>
}