package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class PracticeClimberSubsystem extends SubsystemBase {
    SparkMax climbMotor = new SparkMax(Constants.PracticeClimberConstants.kClimberMotorId, MotorType.kBrushless);
    SparkMaxConfig climbMotorConfig = new SparkMaxConfig();

    RelativeEncoder encoder = climbMotor.getEncoder();

    public PracticeClimberSubsystem() {
        //climbMotorConfig.inverted =  Constants.PracticeClimberConstants.kClimberMotorIsInverted;
        climbMotorConfig.idleMode(IdleMode.kBrake);

        climbMotor.configure(climbMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public Command climb() {
        return Commands.runOnce(() -> {
            climbMotor.set(Constants.PracticeClimberConstants.kClimbSpeed); // turn on motor
            while(encoder.getPosition() < Constants.PracticeClimberConstants.kFinalClimbPosition) {} //wait until motor has turned all the way for climb
            climbMotor.set(0); // stop motor
        }, this).withName("PracticeClimber.Climb");
    }

    public Command lower() {
        return Commands.runOnce(() -> {
            climbMotor.set(-Constants.PracticeClimberConstants.kClimbSpeed); // turn on motor
            while(encoder.getPosition() < Constants.PracticeClimberConstants.kFinalClimbPosition) {} //wait until motor has turned all the way for climb
            climbMotor.set(0); // stop motor
        }, this).withName("PracticeClimber.Climb");
    }                                                            
}
