import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;


import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.lightingSubsystem;
import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class lightTest {
    lightingSubsystem light;
  @BeforeEach // this method will run before each test
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    light = new lightingSubsystem();

  }

  @Test // marks this method as a test
  void initializationTest() {
    assertNotNull(light);
  }
   

}
