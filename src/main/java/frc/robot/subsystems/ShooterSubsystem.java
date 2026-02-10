package frc.robot.subsystems;

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
import frc.robot.Constants.ClimbingConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

        private SparkMax collumnMotor = new SparkMax(ShooterConstants.kCollumnMotorPort,
                        MotorType.kBrushless);
        private SparkMax shooterMotor = new SparkMax(ShooterConstants.kShooterMotorPort,
                        MotorType.kBrushless);
        private SparkMaxConfig shooterConfig = new SparkMaxConfig();
        private SparkMaxConfig collumnConfig = new SparkMaxConfig();
        private SparkClosedLoopController shooterController = shooterMotor.getClosedLoopController();
        private SparkClosedLoopController collumnController = collumnMotor.getClosedLoopController();
        private RelativeEncoder shooterEncoder;
        private RelativeEncoder collumnEncoder;

        public ShooterSubsystem() {

                shooterConfig
                                .inverted(false)
                                .idleMode(IdleMode.kCoast);

                collumnConfig
                                .inverted(false)
                                .idleMode(IdleMode.kCoast);
                shooterConfig.encoder
                                .positionConversionFactor(1)
                                .velocityConversionFactor(1);

                collumnConfig.encoder
                                .positionConversionFactor(1)
                                .velocityConversionFactor(1);

                shooterConfig.closedLoop
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

                collumnConfig.closedLoop
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

                shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                collumnMotor.configure(collumnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                shooterEncoder = shooterMotor.getEncoder();
                collumnEncoder = collumnMotor.getEncoder();

                SmartDashboard.setDefaultNumber("Shooter/Shooter/Velocity", 0);
                SmartDashboard.setDefaultNumber("Shooter/Collumn/Velocity", 0);
        }

        public void setShooterVelocity(double targetVelocity) {
                shooterController.setReference(targetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        }

        public void setColumnVelocity(double targetVelocity) {
                collumnController.setReference(targetVelocity, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }

        @Override
        public void periodic() {
                SmartDashboard.putNumber("Shooter/Shooter/Velocity", shooterEncoder.getVelocity());
                SmartDashboard.putNumber("Shooter/Collumn/Velocity", collumnEncoder.getVelocity());

        }

}