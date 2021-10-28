package kyberlib

import java.io.File
import java.io.IOException

fun String.runCommand(workingDir: File) {
    try {
        val parts = this.split("\\s".toRegex())
        val proc = ProcessBuilder(*parts.toTypedArray()).apply {
            directory(workingDir)
            redirectOutput(ProcessBuilder.Redirect.PIPE)
            redirectError(ProcessBuilder.Redirect.PIPE)
        }
        proc.start()
    } catch(e: IOException) {
        e.printStackTrace()
    }
}