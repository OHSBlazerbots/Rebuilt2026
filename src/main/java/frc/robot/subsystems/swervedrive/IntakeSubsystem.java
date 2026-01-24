package frc.robot.subsystems.swervedrive;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private SparkMax rollerMotor = new SparkMax(IntakeConstants.rollerCanID,
            MotorType.kBrushless);
    private SparkMaxConfig rollerConfig = new SparkMaxConfig();
    private SparkClosedLoopController rollerController = rollerMotor.getClosedLoopController();
    
    private SparkMax pivotMotor = new SparkMax(IntakeConstants.pivotCanID,
            MotorType.kBrushless);
    private SparkMaxConfig pivotConfig = new SparkMaxConfig();
    private SparkClosedLoopController pivotController = pivotMotor.getClosedLoopController();

    private SparkLimitSwitch forwardLimitSwitch;
    private SparkLimitSwitch reverseLimitSwitch;
    private RelativeEncoder rollerEncoder;
    private RelativeEncoder pivotEncoder;

    public IntakeSubsystem() {

        rollerConfig
                .inverted(true)
                .idleMode(IdleMode.kCoast);

        rollerEncoder = rollerMotor.getEncoder();

        rollerConfig.limitSwitch
                .forwardLimitSwitchEnabled(false)
                .reverseLimitSwitchEnabled(false);
        
        
                pivotConfig
                .inverted(true)
                .idleMode(IdleMode.kBrake);

        forwardLimitSwitch = pivotMotor.getForwardLimitSwitch();
        reverseLimitSwitch = pivotMotor.getReverseLimitSwitch();
        pivotEncoder = pivotMotor.getEncoder();

        pivotConfig.limitSwitch
                .forwardLimitSwitchType(Type.kNormallyOpen)
                .forwardLimitSwitchEnabled(true)
                .reverseLimitSwitchType(Type.kNormallyOpen)
                .reverseLimitSwitchEnabled(true);
        

        /*
         * Configure the encoder. For this specific example, we are using the
         * integrated encoder of the NEO, and we don't need to configure it. If
         * needed, we can adjust values like the position or velocity conversion
         * factors.
         */
        rollerConfig.encoder
                .positionConversionFactor(1)
                .velocityConversionFactor(1);
        pivotConfig.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);        

        /*
         * Configure the closed loop controller. We want to make sure we set the
         * feedback sensor as the primary encoder.
         */
        rollerConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control. We don't need to pass a closed loop
                // slot, as it will default to slot 0.
                .p(0.1)
                .i(0)
                .d(0)
                .outputRange(-1, 1)
                // Set PID values for velocity control in slot 1
                .p(0.0001, ClosedLoopSlot.kSlot1)
                .i(0, ClosedLoopSlot.kSlot1)
                .d(0, ClosedLoopSlot.kSlot1)
                .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1);
        pivotConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control. We don't need to pass a closed loop
                // slot, as it will default to slot 0.
                .p(0.1)
                .i(0)
                .d(0)
                .outputRange(-1, 1)
                // Set PID values for velocity control in slot 1
                .p(0.0001, ClosedLoopSlot.kSlot1)
                .i(0, ClosedLoopSlot.kSlot1)
                .d(0, ClosedLoopSlot.kSlot1)
                .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pivotEncoder.setPosition(0);
        // SmartDashboard.setDefaultBoolean("Intake/direction", true);
        // SmartDashboard.setDefaultNumber("Intake/Target Position", 0);
        // SmartDashboard.setDefaultNumber("Intake/Target Velocity", 0);
        // SmartDashboard.setDefaultBoolean("Intake/Control Mode", false);
        // SmartDashboard.setDefaultBoolean("Intake/Reset Encoder", false);
    }

    // public void setIntakeVelocity(double targetVelocity) {
    //     rollerController.setReference(targetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    //     System.out.println("trying to set motor to speed " + targetVelocity);
    // }

    // public void setIntakePosition(double targetPosition) {
    //     rollerController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    // }

  //  @Override
    // public void periodic() {
    //     // Display data from SPARK onto the dashboard
    //     SmartDashboard.putBoolean("Intake/Forward Limit Reached", forwardLimitSwitch.isPressed());
    //     SmartDashboard.putNumber("Intake/Applied Output", rollerMotor.getAppliedOutput());
    //     SmartDashboard.putNumber("Intake/Position", encoder.getPosition());

    //     SmartDashboard.putNumber("Intake/PrimaryMotor set output", rollerMotor.get());

    //     SmartDashboard.putNumber("Intake/Actual Position", encoder.getPosition());
    //     SmartDashboard.putNumber("Elevator/Actual Velocity", encoder.getVelocity());

    //     if (SmartDashboard.getBoolean("Intake/Reset Encoder", false)) {
    //         SmartDashboard.putBoolean("Intake/Reset Encoder", false);
    //         // Reset the encoder position to 0
    //         encoder.setPosition(0);
    //     }
    // }

}    
    

