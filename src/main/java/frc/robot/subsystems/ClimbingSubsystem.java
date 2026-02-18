// package frc.robot.subsystems;

// import frc.robot.Constants.ClimbingConstants;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.ClosedLoopSlot;
// import com.revrobotics.spark.FeedbackSensor;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkLimitSwitch;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.LimitSwitchConfig.Type;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkMaxConfig;

// public class ClimbingSubsystem extends SubsystemBase {

//         private SparkMax m_PrimaryMotor = new SparkMax(ClimbingConstants.kRightClimbingMotorPort,
//                         MotorType.kBrushless);
//         private SparkMax m_SecondaryMotor = new SparkMax(ClimbingConstants.kLeftClimbingMotorPort,
//                         MotorType.kBrushless);
//         private SparkMaxConfig primaryConfig = new SparkMaxConfig();
//         private SparkMaxConfig secondaryConfig = new SparkMaxConfig();
//         private SparkClosedLoopController m_ClimbingPrimaryController = m_PrimaryMotor.getClosedLoopController();

//         private SparkLimitSwitch forwardLimitSwitch;
//         private SparkLimitSwitch reverseLimitSwitch;
//         private RelativeEncoder encoder;

//         public ClimbingSubsystem() {

//                 primaryConfig
//                                 .inverted(true)
//                                 .idleMode(IdleMode.kBrake);

//                 secondaryConfig.follow(m_PrimaryMotor.getDeviceId(), true);

//                 forwardLimitSwitch = m_PrimaryMotor.getForwardLimitSwitch();
//                 reverseLimitSwitch = m_PrimaryMotor.getReverseLimitSwitch();
//                 encoder = m_PrimaryMotor.getEncoder();

//                 primaryConfig.limitSwitch
//                                 .forwardLimitSwitchType(Type.kNormallyOpen)
//                                 .forwardLimitSwitchEnabled(true)
//                                 .reverseLimitSwitchType(Type.kNormallyOpen)
//                                 .reverseLimitSwitchEnabled(true);

//                 primaryConfig.softLimit
//                                 .forwardSoftLimit(ClimbingConstants.kForwardSoftLimitRotations)
//                                 .forwardSoftLimitEnabled(true)
//                                 .reverseSoftLimit(ClimbingConstants.kReverseSoftLimitRotations)
//                                 .reverseSoftLimitEnabled(true);

//                 /*
//                  * Configure the encoder. For this specific example, we are using the
//                  * integrated encoder of the NEO, and we don't need to configure it. If
//                  * needed, we can adjust values like the position or velocity conversion
//                  * factors.
//                  */
//                 primaryConfig.encoder
//                                 .positionConversionFactor(1)
//                                 .velocityConversionFactor(1);

//                 /*
//                  * Configure the closed loop controller. We want to make sure we set the
//                  * feedback sensor as the primary encoder.
//                  */
//                 primaryConfig.closedLoop
//                                 .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
//                                 .p(0.1)
//                                 .i(0)
//                                 .d(0)
//                                 .outputRange(-1, 1)
//                                 .p(0.0001, ClosedLoopSlot.kSlot1)
//                                 .i(0, ClosedLoopSlot.kSlot1)
//                                 .d(0, ClosedLoopSlot.kSlot1)
//                                 .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
//                                 .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

//                 m_PrimaryMotor.configure(primaryConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//                 m_SecondaryMotor.configure(secondaryConfig, ResetMode.kResetSafeParameters,
//                                 PersistMode.kPersistParameters);

//                 encoder.setPosition(0);
//                 SmartDashboard.setDefaultBoolean("Climbing/direction", true);
//                 SmartDashboard.setDefaultNumber("Climbing/Target Position", 0);
//                 SmartDashboard.setDefaultNumber("Climbing/Target Velocity", 0);
//                 SmartDashboard.setDefaultBoolean("Climbing/Control Mode", false);
//                 SmartDashboard.setDefaultBoolean("Climbing/Reset Encoder", false);
//         }

//         public void setClimbingVelocity(double targetVelocity) {
//                 m_ClimbingPrimaryController.setReference(targetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
//         }

//         public void setClimbingPosition(double targetPosition) {
//                 m_ClimbingPrimaryController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
//         }

//         @Override
//         public void periodic() {
//                 // Display data from SPARK onto the dashboard
//                 SmartDashboard.putBoolean("Climbing/Forward Limit Reached", forwardLimitSwitch.isPressed());
//                 SmartDashboard.putBoolean("Climbing/Reverse Limit Reached", reverseLimitSwitch.isPressed());
//                 SmartDashboard.putNumber("Climbing/Applied Output", m_PrimaryMotor.getAppliedOutput());
//                 SmartDashboard.putNumber("Climbing/Position", encoder.getPosition());

//                 SmartDashboard.putNumber("Climbing/PrimaryMotor set output", m_PrimaryMotor.get());
//                 SmartDashboard.putNumber("Climbing/SecondaryMotor set output", m_SecondaryMotor.get());

//                 SmartDashboard.putNumber("Climbing/Actual Position", encoder.getPosition());
//                 SmartDashboard.putNumber("Climbing/Actual Velocity", encoder.getVelocity());

//                 if (SmartDashboard.getBoolean("Climbing/Reset Encoder", false)) {
//                         SmartDashboard.putBoolean("Climbing/Reset Encoder", false);
//                         // Reset the encoder position to 0
//                         encoder.setPosition(0);
//                 }
//         }

// }
