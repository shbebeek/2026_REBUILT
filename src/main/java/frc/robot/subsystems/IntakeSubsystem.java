package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakeSubsystem extends SubsystemBase{
    private SparkMax rollerController = new SparkMax(Constants.IntakeConstants.kRollerMotorId,MotorType.kBrushless);
    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.OPEN_LOOP)
        .withTelemetry("IntakeRollerMotor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(1))) // Direct drive, adjust if geared
        .withMotorInverted(true)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(40));

    private SmartMotorController smc = new SparkWrapper(rollerController,DCMotor.getNEO(1),smcConfig);

    private final FlyWheelConfig intakeConfig = new FlyWheelConfig(smc)
        .withDiameter(Inches.of(4))
        .withMass(Pounds.of(0.5))
        .withUpperSoftLimit(RPM.of(6000))
        .withLowerSoftLimit(RPM.of(-6000))
        .withTelemetry("IntakeRoller", TelemetryVerbosity.HIGH);

    private FlyWheel intake = new FlyWheel(intakeConfig);

    // 5:1, 5:1, 60/18 reduction (cranberry)
    private SmartMotorControllerConfig intakePivotSmartMotorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(25,0,0,DegreesPerSecond.of(360),DegreesPerSecondPerSecond.of(360))
        .withTelemetry("IntakePivotMotor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 5, 6, 60.0 / 18.0)))
        // .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 5, 60.0 / 18.0, 42)))
        .withMotorInverted(false)
        .withIdleMode(MotorMode.COAST)
        .withSoftLimit(Degrees.of(0), Degrees.of(150))
        .withStatorCurrentLimit(Amps.of(10))
        .withClosedLoopRampRate(Seconds.of(0.1))
        .withOpenLoopRampRate(Seconds.of(0.1));
    
    private SparkMax pivotMotor = new SparkMax(Constants.IntakeConstants.kPivotMotorId, MotorType.kBrushless);

    private SmartMotorController intakePivotController = new SparkWrapper(pivotMotor, DCMotor.getNEO(1),intakePivotSmartMotorConfig);

    private final ArmConfig intakePivotConfig = new ArmConfig(intakePivotController)
        .withSoftLimits(Degrees.of(0), Degrees.of(150))
        .withHardLimit(Degrees.of(0), Degrees.of(155))
        .withStartingPosition(Degrees.of(0))
        .withLength(Feet.of(1))
        .withMass(Pounds.of(2))
        .withTelemetry("IntakePivot", TelemetryVerbosity.HIGH);
    
    private Arm intakePivot = new Arm(intakePivotConfig);

    public IntakeSubsystem(){
        
    }
}
