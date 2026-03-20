import static org.junit.jupiter.api.Assertions.assertNotNull;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class SwerveTest {
  //   static final double DELTA = 1e-2; // acceptable deviation range
  SwerveSubsystem drivebase;

  @BeforeEach // this method will run before each test
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
  }

  @Test // marks this method as a test
  void poseEstimateTest() {
    drivebase.periodic();
    assertNotNull(drivebase.getPose());
  }
}
