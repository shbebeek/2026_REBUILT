package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class PracticeIntake extends SubsystemBase{
    private static TalonFX intakeMotor = new TalonFX(Constants.PracticeIntakeConstants.kRollerMotorId);
    private static TalonFX pivotMotor = new TalonFX(Constants.PracticeIntakeConstants.kPivotMotorId);

    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    // Invert: CounterClockwise_Positive (default) or Clockwise_Positive
    

    public PracticeIntake() {
        intakeMotor.setNeutralMode(NeutralModeValue.Coast);
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);

        // intakeConfig.MotorOutput.Inverted = InvertValue.Clockwise_Positive;
        // intakeConfig.MotorOutput.Inverted = InvertValue.Clockwise_Positive;
        
        intakeMotor.getConfigurator().apply(intakeConfig);
        pivotMotor.getConfigurator().apply(pivotConfig);
    }

    public Command intake() {
        return Commands.runOnce(() -> intakeMotor.set(Constants.PracticeIntakeConstants.kIntakeSpeed), this).withName("PracticeIntake.Intake");
    }

    public Command outTake() {
        return Commands.runOnce(() -> intakeMotor.set(-Constants.PracticeIntakeConstants.kIntakeSpeed), this).withName("PracticeIntake.Outtake");
    }

    public Command Deploy() {
        return Commands.runOnce(() ->{
            pivotMotor.setControl(new PositionVoltage(Constants.PracticeIntakeConstants.kDeployMotorRotations));
        }, this).withName("PracticeIntake.Deploy");
    }

    public Command Retract() {
        return Commands.runOnce(() ->{
            pivotMotor.setControl(new PositionVoltage(-Constants.PracticeIntakeConstants.kDeployMotorRotations));
        }, this).withName("PracticeIntake.Retract");
    }
}
