package frc.team254.drivers;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

/**
 * This class is a thin wrapper around the CANTalon that reduces CAN bus / CPU overhead by skipping duplicate set
 * commands. (By default the Talon flushes the Tx buffer on every set call).
 */
public class LazyTalonFX extends TalonFX {
    protected double mLastSet = Double.NaN;
    protected TalonFXControlMode mLastControlMode = null;

    public LazyTalonFX(int deviceNumber) {
        super(deviceNumber);
    }

    public LazyTalonFX(int driveID, String canivoreBusName) {
        super(driveID, canivoreBusName);
    }

    public double getLastSet() {
        return mLastSet;
    }

    @Override
    public void set(TalonFXControlMode mode, double value) {
        if (value != mLastSet || mode != mLastControlMode) {
            mLastSet = value;
            mLastControlMode = mode;
            super.set(mode, value);
        }
    }
}