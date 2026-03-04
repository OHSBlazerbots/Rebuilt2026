import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.FeederSubsystem;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class feederTest {
  FeederSubsystem feeder;

  @BeforeEach // this method will run before each test
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    feeder = new FeederSubsystem();
  }

  @Test // marks this method as a test
  void poseEstimateTest() {
    feeder.setRollerVelocity(67);
    // assertNotNull(feeder.setPose());
  }
}
