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
import frc.robot.Constants.ShooterConstants;
import frc.robot.Components.LinearServo;

public class ShooterSubsystem extends SubsystemBase {

        // private SparkMax collumnMotor = new SparkMax(ShooterConstants.kCollumnMotorPort, MotorType.kBrushless);
        private SparkMax shooterMotor = new SparkMax(ShooterConstants.kShooterMotorPort, MotorType.kBrushless);
        // private SparkMax shooterLeftMotor = new
        // SparkMax(ShooterConstants.kShooterMotorPort, MotorType.kBrushless);
        // private SparkMax shooterMiddleMotor = new
        // SparkMax(ShooterConstants.kShooterMotorPort, MotorType.kBrushless);
        // private SparkMax shooterRightMotor = new
        // SparkMax(ShooterConstants.kShooterMotorPort, MotorType.kBrushless);
        private SparkMaxConfig shooterConfig = new SparkMaxConfig();
        // private SparkMaxConfig shooterLeftConfig = new SparkMaxConfig();
        // private SparkMaxConfig shooterMiddleConfig = new SparkMaxConfig();
        // private SparkMaxConfig shooterRightConfig = new SparkMaxConfig();
        // private SparkMaxConfig collumnConfig = new SparkMaxConfig();
        private SparkClosedLoopController shooterController = shooterMotor.getClosedLoopController();
        // private SparkClosedLoopController shooterLeftController =
        // shooterLeftMotor.getClosedLoopController();
        // private SparkClosedLoopController shooterMiddleController =
        // shooterMiddleMotor.getClosedLoopController();
        // private SparkClosedLoopController shooterRightController =
        // shooterRightMotor.getClosedLoopController();
        // private SparkClosedLoopController collumnController = collumnMotor.getClosedLoopController();
        private RelativeEncoder shooterEncoder;
        // private RelativeEncoder shooterLeftEncoder;
        // private RelativeEncoder shooterMiddleEncoder;
        // private RelativeEncoder shooterRightEncoder;
        // private RelativeEncoder collumnEncoder;

        // Initialize LinearServo
        private LinearServo linearServo;

        public ShooterSubsystem() {
                shooterConfig.inverted(false).idleMode(IdleMode.kCoast);
                // shooterLeftConfig.inverted(false).idleMode(IdleMode.kCoast);
                // shooterMiddleConfig.inverted(false).idleMode(IdleMode.kCoast);
                // shooterRightConfig.inverted(false).idleMode(IdleMode.kCoast);
                // collumnConfig.inverted(false).idleMode(IdleMode.kCoast);

                shooterConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
                // shooterLeftConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
                // shooterMiddleConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
                // shooterRightConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
                // collumnConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);

                shooterConfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .p(0.1)
                                .i(0)
                                .d(0)
                                .outputRange(-1, 1)
                                .p(0.0001, ClosedLoopSlot.kSlot1)
                                .i(0, ClosedLoopSlot.kSlot1)
                                .d(0, ClosedLoopSlot.kSlot1)
                                .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
                                .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

                // shooterLeftConfig.closedLoop
                // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // .p(0.1)
                // .i(0)
                // .d(0)
                // .outputRange(-1, 1)
                // .p(0.0001, ClosedLoopSlot.kSlot1)
                // .i(0, ClosedLoopSlot.kSlot1)
                // .d(0, ClosedLoopSlot.kSlot1)
                // .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
                // .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

                // shooterMiddleConfig.closedLoop
                // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // .p(0.1)
                // .i(0)
                // .d(0)
                // .outputRange(-1, 1)
                // .p(0.0001, ClosedLoopSlot.kSlot1)
                // .i(0, ClosedLoopSlot.kSlot1)
                // .d(0, ClosedLoopSlot.kSlot1)
                // .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
                // .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

                // shooterRightConfig.closedLoop
                // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // .p(0.1)
                // .i(0)
                // .d(0)
                // .outputRange(-1, 1)
                // .p(0.0001, ClosedLoopSlot.kSlot1)
                // .i(0, ClosedLoopSlot.kSlot1)
                // .d(0, ClosedLoopSlot.kSlot1)
                // .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
                // .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

                // collumnConfig.closedLoop
                //                 .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                //                 .p(0.1)
                //                 .i(0)
                //                 .d(0)
                //                 .outputRange(-1, 1)
                //                 .p(0.0001, ClosedLoopSlot.kSlot1)
                //                 .i(0, ClosedLoopSlot.kSlot1)
                //                 .d(0, ClosedLoopSlot.kSlot1)
                //                 .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
                //                 .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

                shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                // shooterLeftMotor.configure(shooterLeftConfig, ResetMode.kResetSafeParameters,
                // PersistMode.kPersistParameters);
                // shooterMiddleMotor.configure(shooterMiddleConfig,
                // ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                // shooterRightMotor.configure(shooterRightConfig,
                // ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                // collumnMotor.configure(collumnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                shooterEncoder = shooterMotor.getEncoder();
                // shooterLeftEncoder = shooterLeftMotor.getEncoder();
                // shooterMiddleEncoder = shooterMiddleMotor.getEncoder();
                // shooterRightEncoder = shooterRightMotor.getEncoder();
                // collumnEncoder = collumnMotor.getEncoder();

                linearServo = new LinearServo(0, 0, 0);

                SmartDashboard.setDefaultNumber("Shooter/Shooter/Velocity", 0);
                // SmartDashboard.setDefaultNumber("Shooter/Collumn/Velocity", 0);
        }

        public void setShooterVelocity(double targetVelocity) {
                shooterController.setReference(targetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        }

        // public void setShooterLeftVelocity(double targetVelocity) {
        // shooterLeftController.setReference(targetVelocity, ControlType.kVelocity,
        // ClosedLoopSlot.kSlot1);
        // }

        // public void setShooterMiddleVelocity(double targetVelocity) {
        // shooterMiddleController.setReference(targetVelocity, ControlType.kVelocity,
        // ClosedLoopSlot.kSlot1);
        // }

        // public void setShooterRightVelocity(double targetVelocity) {
        // shooterRightController.setReference(targetVelocity, ControlType.kVelocity,
        // ClosedLoopSlot.kSlot1);
        // }

        // public void setColumnVelocity(double targetVelocity) {
        //         // collumnController.setReference(targetVelocity, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        // }

        // Set the position of the linear servo
        public void setLinearServoPosition(double targetPosition) {
                linearServo.setPosition(targetPosition);
        }


        @Override
        public void periodic() {
                SmartDashboard.putNumber("Shooter/Shooter/Velocity", shooterEncoder.getVelocity());
                // SmartDashboard.putNumber("Shooter/Shooter Left/Velocity",
                // shooterLeftEncoder.getVelocity());
                // SmartDashboard.putNumber("Shooter/Shooter Middle/Velocity",
                // shooterMiddleEncoder.getVelocity());
                // SmartDashboard.putNumber("Shooter/Shooter Right/Velocity",
                // shooterRightEncoder.getVelocity());
                // SmartDashboard.putNumber("Shooter/Collumn/Velocity", collumnEncoder.getVelocity());
        }
}
