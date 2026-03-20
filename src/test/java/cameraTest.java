import static org.junit.jupiter.api.Assertions.assertNotNull;

import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.DriverCameraSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class cameraTest {
  IntakeSubsystem intake;
  DriverCameraSubsystem camera;

  @BeforeEach // this method will run before each test
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    camera = new DriverCameraSubsystem();
  }

  @Test // marks this method as a test
  void initializationTest() {
    assertNotNull(camera);
  }
}
