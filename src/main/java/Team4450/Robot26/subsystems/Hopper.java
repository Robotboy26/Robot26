package Team4450.Robot26.subsystems;

import Team4450.Robot26.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Hopper extends SubsystemBase {
    private final TalonFX hopperMotor = new TalonFX(Constants.HOPPER_MOTOR_CAN_ID);

    private double targetRpm = 0.0;

    public Hopper() {
        // Configure motor neutral mode
        hopperMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setTargetRpm(double rpm) {
        this.targetRpm = rpm;
        double targetRps = targetRpm / 60.0; // Convert RPM to RPS
        hopperMotor.set(targetRps);
    }

    public double getTargetRpm() {
        return this.targetRpm;
    }

    public void stop() {
        setTargetRpm(0.0);
    }
}