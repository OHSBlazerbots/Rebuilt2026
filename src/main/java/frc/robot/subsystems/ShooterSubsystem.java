package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.FeedbackSensor;
// import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.Timer;

public class ShooterSubsystem extends SubsystemBase {

        private SparkFlex kickerMotor = new SparkFlex(ShooterConstants.kKickerMotorPort, MotorType.kBrushless);
        private SparkFlex shooterLeftMotor = new SparkFlex(ShooterConstants.kShooterLeftMotorPort,
                        MotorType.kBrushless);
        private SparkFlex shooterMiddleMotor = new SparkFlex(ShooterConstants.kShooterMiddleMotorPort,
                        MotorType.kBrushless);
        private SparkFlex shooterRightMotor = new SparkFlex(ShooterConstants.kShooterRightMotorPort,
                        MotorType.kBrushless);
        private SparkFlex angleMakerMotor = new SparkFlex(ShooterConstants.kAngleMakerPort, MotorType.kBrushless);

        private SparkFlexConfig shooterConfig = new SparkFlexConfig();
        private SparkFlexConfig leftConfig = new SparkFlexConfig();
        private SparkFlexConfig middleConfig = new SparkFlexConfig();
        private SparkFlexConfig rightConfig = new SparkFlexConfig();
        private SparkFlexConfig kickerConfig = new SparkFlexConfig();
        private SparkFlexConfig angleMakerConfig = new SparkFlexConfig();

        private SparkClosedLoopController shooterLeftController = shooterLeftMotor.getClosedLoopController();
        private SparkClosedLoopController shooterRightController = shooterRightMotor.getClosedLoopController();
        private SparkClosedLoopController shooterMiddleController = shooterMiddleMotor.getClosedLoopController();

        private SparkClosedLoopController kickerController = kickerMotor.getClosedLoopController();
        private SparkClosedLoopController angleMakerController = angleMakerMotor.getClosedLoopController();

        private RelativeEncoder shooterLeftEncoder;
        private RelativeEncoder shooterMiddleEncoder;
        private RelativeEncoder shooterRightEncoder;
        private RelativeEncoder kickerEncoder;
        private RelativeEncoder angleMakerEncoder;

        // Initialize LinearServo
        // private LinearServo linearServo;

        public enum ShooterState {
                IDLE, SPIN_UP, READY, FIRING, RECOVERY
        }

        private ShooterState state = ShooterState.IDLE;
        private double currentTarget = 0;
        private final Timer firingTimer = new Timer();

        public ShooterSubsystem() {
                shooterConfig.inverted(false).idleMode(IdleMode.kCoast);
                kickerConfig.inverted(false).idleMode(IdleMode.kCoast);
                angleMakerConfig.inverted(false).idleMode(IdleMode.kCoast);

                shooterConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
                kickerConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
                angleMakerConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);

                shooterConfig.closedLoop
                                .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .p(0.0001, ClosedLoopSlot.kSlot1)
                                .i(0, ClosedLoopSlot.kSlot1)
                                .d(0, ClosedLoopSlot.kSlot1).feedForward
                                // .kS(0, ClosedLoopSlot.kSlot1)
                                .kV(1 / 5767, ClosedLoopSlot.kSlot1);
                angleMakerConfig.closedLoop
                                .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .p(0.0001, ClosedLoopSlot.kSlot1)
                                .i(0, ClosedLoopSlot.kSlot1)
                                .d(0, ClosedLoopSlot.kSlot1).feedForward
                                // .kS(0, ClosedLoopSlot.kSlot1)
                                .kV(1 / 5767, ClosedLoopSlot.kSlot1);
                kickerConfig.closedLoop
                                .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .p(0.0001, ClosedLoopSlot.kSlot1)
                                .i(0, ClosedLoopSlot.kSlot1)
                                .d(0, ClosedLoopSlot.kSlot1).feedForward
                                // .kS(0, ClosedLoopSlot.kSlot1)
                                .kV(1 / 5767, ClosedLoopSlot.kSlot1);


                middleConfig
                                .apply(shooterConfig);
                rightConfig
                                .apply(shooterConfig);
                leftConfig
                                .apply(shooterConfig);              

                shooterLeftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                shooterMiddleMotor.configure(middleConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
                shooterRightMotor.configure(rightConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
                kickerMotor.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                angleMakerMotor.configure(angleMakerConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);

                shooterLeftEncoder = shooterLeftMotor.getEncoder();
                shooterMiddleEncoder = shooterMiddleMotor.getEncoder();
                shooterRightEncoder = shooterRightMotor.getEncoder();
                kickerEncoder = kickerMotor.getEncoder();
                angleMakerEncoder = angleMakerMotor.getEncoder();

                // linearServo = new LinearServo(0, 0, 0);

                SmartDashboard.setDefaultNumber("Shooter/Shooter Left/Velocity", 0);
                SmartDashboard.setDefaultNumber("Shooter/Shooter Middle/Velocity", 0);
                SmartDashboard.setDefaultNumber("Shooter/Shooter Right/Velocity", 0);
                SmartDashboard.setDefaultNumber("Shooter/Kicker/Velocity", 0);
                SmartDashboard.setDefaultNumber("Shooter/Angle Maker/Velocity", 0);
                SmartDashboard.setDefaultBoolean("Shooter/ Shooter Ready", false);

        }

        public void requestShot(double targetRPM) {
                if (state == ShooterState.IDLE) {
                        currentTarget = targetRPM;
                        setShooterVelocity(currentTarget);
                        setAngleMakerVelocity(ShooterConstants.angleRPM);
                        state = ShooterState.SPIN_UP;
                }
        }

        public void cancelShot() {
                state = ShooterState.IDLE;
                stopShooter();
                stopKicker();
        }

        public ShooterState getState() {
                return state;
        }

        public void setShooterVelocity(double targetShooterVelocity) {
                shooterLeftController.setSetpoint(targetShooterVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
                shooterMiddleController.setSetpoint(targetShooterVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
                shooterRightController.setSetpoint(-targetShooterVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);

        }

        public void setKickerVelocity(double targetVelocity) {
                kickerController.setSetpoint(targetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        }

        public void setAngleMakerVelocity(double targetVelocity) {
                angleMakerController.setSetpoint(targetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        }

        public void stopShooter() {
                shooterLeftMotor.stopMotor();
                shooterRightMotor.stopMotor();
                shooterMiddleMotor.stopMotor();
                angleMakerMotor.stopMotor();

        }

        public void trenchShot() {
                setShooterVelocity(ShooterConstants.trenchRPM);
                setAngleMakerVelocity(ShooterConstants.angleRPM);
        }

        public void trenchShotReversed() {
                setShooterVelocity(-ShooterConstants.trenchRPM);
                setAngleMakerVelocity(-ShooterConstants.angleRPM);
        }

        public void closeShot() {
                setShooterVelocity(ShooterConstants.hubRPM);
                setAngleMakerVelocity(ShooterConstants.angleRPM);
        }

        public void startShooter() {
                setShooterVelocity(ShooterConstants.fullPower);
        }

        public void runBackwards() {
                setShooterVelocity(-ShooterConstants.fullPower);
        }


        public void stopKicker() {
                kickerMotor.stopMotor();
        }

        public void startAngleMaker() {
                setAngleMakerVelocity(ShooterConstants.angleRPM);
        }

        public void backwardAngleMaker() {
                setAngleMakerVelocity(-ShooterConstants.angleRPM);
        }

        // Set the position of the linear servo
        // public void setLinearServoPosition(double targetPosition) {
        // linearServo.setPosition(targetPosition);
        // }

        @Override
        public void periodic() {
                switch (state) {
                        case IDLE:
                                break;
                        case SPIN_UP:
                                if (isAtSetpoint(currentTarget)) {
                                        state = ShooterState.READY;
                                }
                                break;
                        case READY:
                                // Fire is triggered externally via fireShot()
                                // cancelShot() returns to IDLE
                                break;
                        case FIRING:
                                if (firingTimer.hasElapsed(ShooterConstants.firingDuration)) {
                                        stopKicker();
                                        state = ShooterState.RECOVERY;
                                }
                                break;
                        case RECOVERY:
                                if (isAtSetpoint(currentTarget)) {
                                        state = ShooterState.READY;
                                }
                                break;
                }

                SmartDashboard.putNumber("Shooter/Shooter Left/Velocity", shooterLeftEncoder.getVelocity());
                SmartDashboard.putNumber("Shooter/Shooter Middle/Velocity", shooterMiddleEncoder.getVelocity());
                SmartDashboard.putNumber("Shooter/Shooter Right/Velocity", shooterRightEncoder.getVelocity());
                SmartDashboard.putNumber("Shooter/Kicker/Velocity", kickerEncoder.getVelocity());
                SmartDashboard.putNumber("Shooter/Angle Maker/Velocity", angleMakerEncoder.getVelocity());
                SmartDashboard.putBoolean("Shooter/ Shooter Ready", isAtSetpoint(currentTarget));

        }

        public boolean isAtSetpoint(double shooterTarget) {
                return (Math.abs(shooterLeftEncoder.getVelocity() - shooterTarget) <= ShooterConstants.rpmTolerance
                                &&
                                Math.abs(angleMakerEncoder.getVelocity()
                                                - ShooterConstants.angleRPM) <= ShooterConstants.rpmTolerance);
        }

}