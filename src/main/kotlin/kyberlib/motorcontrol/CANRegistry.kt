package kyberlib.motorcontrol

typealias CANKey = String
typealias CANId = Int

val CANRegistry = mutableMapOf<CANKey, CANId>()
