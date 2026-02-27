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
import frc.robot.Constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {

    private SparkMax rollerMotor = new SparkMax(FeederConstants.primaryCanID,
            MotorType.kBrushless);
    private SparkMaxConfig rollerConfig = new SparkMaxConfig();
    private SparkClosedLoopController rollerController = rollerMotor.getClosedLoopController();
    
  
    private RelativeEncoder rollerEncoder;


    public FeederSubsystem() {

        rollerConfig
                .inverted(true)
                .idleMode(IdleMode.kCoast);

        rollerEncoder = rollerMotor.getEncoder();

        
        
                
        
        /*
         * Configure the encoder. For this specific example, we are using the
         * integrated encoder of the NEO, and we don't need to configure it. If
         * needed, we can adjust values like the position or velocity conversion
         * factors.
         */
        rollerConfig.encoder
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
        
        rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       
     
        SmartDashboard.setDefaultNumber("Feeder/Velocity", 0);
    }

    
    public void setRollerVelocity(double targetVelocity) {
        rollerController.setReference(targetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    }

   @Override
    public void periodic() {
        // Display data from SPARK onto the dashboard

        SmartDashboard.putNumber("Feeder/Velocity", rollerEncoder.getVelocity());
        SmartDashboard.putNumber("Feeder/Applied Output", rollerMotor.getAppliedOutput());
      
    }

}    
    

