package Team4450.Robot26.utility;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import java.util.List;

public class LinkedMotors {
    private final TalonFX masterMotor;
    private final List<TalonFX> slaveMotors;
    
    public LinkedMotors(TalonFX master, TalonFX... slaves) {
        this.masterMotor = master;
        this.slaveMotors = List.of(slaves);
    }

    /**
     * Sets the power for all linked motors.
     * @param power The power level to set.
     */
    public void setPower(double power) {
        this.masterMotor.set(power);
        for (TalonFX slaveMotor : this.slaveMotors) {
            slaveMotor.set(power);
        }
    }

    /**
     * Applies a control request to all linked motors.
     * @param req The control request to apply.
     * @param shooter Whether the motors are part of the shooter system.
     */
    public void applyControl(ControlRequest req, boolean shooter) {
        this.masterMotor.setControl(req);
        if (shooter) {
            this.slaveMotors.get(0).setControl(new Follower(this.masterMotor.getDeviceID(), MotorAlignmentValue.Opposed));
            this.slaveMotors.get(1).setControl(new Follower(this.masterMotor.getDeviceID(), MotorAlignmentValue.Aligned));
            this.slaveMotors.get(2).setControl(new Follower(this.masterMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        }
    }

    /**
     * Gets the total number of linked motors.
     * @return The total number of motors.
     */
    public int getTotalMotors() {
        return this.slaveMotors.size() + 1;
    }

    /**
     * Gets a motor by its index.
     * @param i The index of the motor.
     * @return The motor at the specified index, or null if out of bounds.
     */
    public TalonFX getMotorByIndex(int i) {
        if (i < 0 || i >= this.slaveMotors.size() + 1) {
            return null;
        }
        return i == 0 ? this.masterMotor : this.slaveMotors.get(i - 1);
    }
}
