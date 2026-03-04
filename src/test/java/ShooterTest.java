import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.ShooterSubsystem;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

// test imcomplete
class ShooterTest {
  ShooterSubsystem m_ShooterSubsystem;

  @BeforeEach // this method will run before each test
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    m_ShooterSubsystem = new ShooterSubsystem();
  }

  @Test
  void shooterTest() {
    m_ShooterSubsystem.setColumnVelocity(67);
    m_ShooterSubsystem.setShooterVelocity(67);
    m_ShooterSubsystem.setLinearServoPosition(67);
  }
}
