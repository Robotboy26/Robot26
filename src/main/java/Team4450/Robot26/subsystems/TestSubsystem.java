package Team4450.Robot26.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class TestSubsystem extends SubsystemBase {
    // private SparkFlex testMotor = new SparkFlex(10, MotorType.kBrushless);
    private TalonFX testMotor = new TalonFX(10);
    private SparkFlexConfig testConfig = new SparkFlexConfig();
    final int kUnitsPerRevolution = 2048;

    public TestSubsystem() {
        testConfig.idleMode(IdleMode.kBrake);
        testMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
        // What will the turret need to update over time
    }

    public void start() {
        testMotor.set(SmartDashboard.getNumber("Test Motor Power", 50) / 2048);
        // testMotor.set(1);
    }

    public void stop() {
        testMotor.set(0);
    }
}
