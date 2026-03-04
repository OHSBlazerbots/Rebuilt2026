import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.IntakeSubsystem;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class intakeTest {
  IntakeSubsystem intake;

  @BeforeEach // this method will run before each test
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    intake = new IntakeSubsystem();
  }

  @Test // marks this method as a test
  void poseEstimateTest() {
    intake.setPivotVelocity(.67);
    intake.setRollerVelocity(.67);
    intake.setPivotPosition(.67);

    // assertNotNull(feeder.setPose());
  }
}
