package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Components.LinearServo;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

        private SparkFlex collumnMotor = new SparkFlex(ShooterConstants.kCollumnMotorPort, MotorType.kBrushless);

        private SparkFlex shooterLeftMotor = new
        SparkFlex(ShooterConstants.kShooterLeftMotorPort, MotorType.kBrushless);
       
        
        private SparkFlex shooterMiddleMotor = new
        SparkFlex(ShooterConstants.kShooterMiddleMotorPort, MotorType.kBrushless);
        private SparkFlex shooterRightMotor = new
        SparkFlex(ShooterConstants.kShooterRightMotorPort, MotorType.kBrushless);

        private SparkFlexConfig leftConfig = new SparkFlexConfig();
        private SparkFlexConfig middleConfig = new SparkFlexConfig();
        private SparkFlexConfig rightConfig = new SparkFlexConfig();
        private SparkFlexConfig collumnConfig = new SparkFlexConfig();

        private SparkClosedLoopController shooterLeftController =
                shooterLeftMotor.getClosedLoopController();
        private SparkClosedLoopController shooterMiddleController =
        shooterMiddleMotor.getClosedLoopController();
        private SparkClosedLoopController shooterRightController =
        shooterRightMotor.getClosedLoopController();

         private SparkClosedLoopController collumnController = collumnMotor.getClosedLoopController();

        private RelativeEncoder shooterLeftEncoder;
        private RelativeEncoder shooterMiddleEncoder;
        private RelativeEncoder shooterRightEncoder;
        private RelativeEncoder collumnEncoder;
        private double targetRPM; 


        // Initialize LinearServo
        private LinearServo linearServo;

        public ShooterSubsystem() {
                leftConfig.inverted(false).idleMode(IdleMode.kCoast);
                collumnConfig.inverted(false).idleMode(IdleMode.kCoast);

                leftConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
                collumnConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);

                leftConfig.closedLoop
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

                


                collumnConfig.closedLoop
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

                shooterLeftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                middleConfig.follow(shooterLeftMotor.getDeviceId(), true);
                rightConfig.follow(shooterLeftMotor.getDeviceId(), true);
                shooterMiddleMotor.configure(middleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                shooterRightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                collumnMotor.configure(collumnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                
                shooterLeftEncoder = shooterLeftMotor.getEncoder();
                shooterMiddleEncoder = shooterMiddleMotor.getEncoder();
                shooterRightEncoder = shooterRightMotor.getEncoder();
                collumnEncoder = collumnMotor.getEncoder();

                // linearServo = new LinearServo(0, 0, 0);

                SmartDashboard.setDefaultNumber("Shooter/Shooter Left/Velocity", 0);
                SmartDashboard.setDefaultNumber("Shooter/Shooter Middle/Velocity", 0);
                SmartDashboard.setDefaultNumber("Shooter/Shooter Right/Velocity", 0);
                SmartDashboard.setDefaultNumber("Shooter/Collumn/Velocity", 0);
        }

        public void setShooterVelocity(double targetVelocity) {
                targetRPM= targetVelocity;
                shooterLeftController.setReference(targetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        }

        public void stopShooter(){
                shooterLeftMotor.stopMotor();
                
        }
        public void startShooter(){
                setShooterVelocity(ShooterConstants.fullPower);
        }
        public void runBackwards(){
                setShooterVelocity(-ShooterConstants.fullPower );
        }
        public void stopColumn(){
                collumnMotor.stopMotor();
        }


        public void setColumnVelocity(double targetVelocity) {
                collumnController.setReference(targetVelocity, ControlType.kPosition, ClosedLoopSlot.kSlot0);
         }

        // Set the position of the linear servo
        // public void setLinearServoPosition(double targetPosition) {
        //         linearServo.setPosition(targetPosition);
        // }


        @Override
        public void periodic() {
                SmartDashboard.putNumber("Shooter/Shooter Left/Velocity",
                shooterLeftEncoder.getVelocity());
                SmartDashboard.putNumber("Shooter/Shooter Middle/Velocity",
                shooterMiddleEncoder.getVelocity());
                SmartDashboard.putNumber("Shooter/Shooter Right/Velocity",
                shooterRightEncoder.getVelocity());
                SmartDashboard.putNumber("Shooter/Collumn/Velocity", collumnEncoder.getVelocity());
        }
        public boolean isAtSetpoint(){
                return Math.abs(shooterLeftEncoder.getVelocity()-targetRPM) <= 60.0;
        }
        
        

}
