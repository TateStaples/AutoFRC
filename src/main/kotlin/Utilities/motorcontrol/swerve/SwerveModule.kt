package frc.team6502.kyberlib.motorcontrol.swerve

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import frc.team6502.kyberlib.motorcontrol.MotorPackage
//import sun.jvm.hotspot.oops.CellTypeState.value
/**
class SwerveModule (drive: MotorPackage, turn: MotorPackage){


    /**
     * Setting turn motor power
     * @param p value from -1 to 1
     */
    fun setTurnPower(p: Double) {
        this.turn.changeControlMode(TalonControlMode.PercentVbus)
        this.turn.set(p)
    }

    /**
     * Setting drive motor power
     * @param p value from -1 to 1
     */
    fun setDrivePower(p: Double) {
        drive.set(p)
    }

    /**
     * Getting the turn encoder position
     * @return turn encoder postition
     */
    fun getTurnEncPos(): Double {
        return turn.getEncPosition()
    }

    /**
     * Thank you CD ozrien for this!!!
     * @return
     */
    fun getAbsPos(): Double {
        return (turn.getPulseWidthPosition() and 0xFFF) / 4095.0
    }

    /**
     * Lets reset the turn encoder position to 0
     */
    fun restTurnEnc() {
        this.turn.setEncPosition(0)
    }

    fun setEncPos(d: Int) {
        turn.setEncPosition(d)
    }

    /**
     * Is electrical good? Probably not.... Is the turn encoder connected?
     * @return true if the encoder is connected
     */
    fun isTurnEncConnected(): Boolean {
        return turn.isSensorPresent(FeedbackDevice.CtreMagEncoder_Relative) === FeedbackDeviceStatus.FeedbackStatusPresent
    }

    fun getTurnRotations(): Int {
        return (turn.getEncPosition() / FULL_ROTATION)
    }

    fun getTurnLocation(): Double {
        return turn.getEncPosition() % FULL_ROTATION / FULL_ROTATION
    }

    fun setTurnPIDToSetPoint(setpoint: Double) {
        turn.changeControlMode(TalonControlMode.Position)
        turn.set(setpoint)
    }

    /**
     * Set turn to pos from 0 to 1 using PID
     * @param setLoc location to set to
     */
    fun setTurnLocation(loc: Double) {
        turn.changeControlMode(TalonControlMode.Position)
        var base: Double = getTurnRotations() * FULL_ROTATION
        if (getTurnEncPos() >= 0) {
            if (base + loc * FULL_ROTATION - getTurnEncPos() < -FULL_ROTATION / 2) {
                base += FULL_ROTATION
            } else if (base + loc * FULL_ROTATION - getTurnEncPos() > FULL_ROTATION / 2) {
                base -= FULL_ROTATION
            }
            turn.set(loc * FULL_ROTATION + base)
        } else {
            if (base - (1 - loc) * FULL_ROTATION - getTurnEncPos() < -FULL_ROTATION / 2) {
                base += FULL_ROTATION
            } else if (base - (1 - loc) * FULL_ROTATION - getTurnEncPos() > FULL_ROTATION / 2) {
                base -= FULL_ROTATION
            }
            turn.set(base - (1 - loc) * FULL_ROTATION)
        }
    }

    fun getError(): Double {
        return turn.getError()
    }

    fun stopBoth() {
        setDrivePower(0.0)
        setTurnPower(0.0)
    }

    fun stopDrive() {
        setDrivePower(0.0)
    }

    fun setBreakMode(b: Boolean) {
        drive.enableBrakeMode(b)
    }
}
 */