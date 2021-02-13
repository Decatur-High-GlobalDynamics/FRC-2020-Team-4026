package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.*;
import frc.robot.Constants;

import org.junit.jupiter.api.*;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

public class ClimberSubsystemTest {

  @Test
  public void itShouldInstantiateGivenParams() {

    // Assemble
    WPI_VictorSPX leftClimber = mock(WPI_VictorSPX.class);
    WPI_TalonSRX rightClimber = mock(WPI_TalonSRX.class);
    DigitalInput leftLimit = mock(DigitalInput.class);
    DigitalInput rightLimit = mock(DigitalInput.class);
    DigitalInput rightHookLimit = mock(DigitalInput.class);
    DigitalInput leftHookLimit = mock(DigitalInput.class);

    // Act
    ClimberSubsystem climber =
        new ClimberSubsystem(
            leftClimber, rightClimber, leftLimit, rightLimit, leftHookLimit, rightHookLimit);

    // Assert
    assertTrue(climber instanceof ClimberSubsystem);
  }

  @Test
  public void itShouldNotInstantiateWithoutParams() {

    // Assemble

    // Act
    Exception exception =
        assertThrows(
            IllegalArgumentException.class,
            () -> {
              @SuppressWarnings("unused")
              ClimberSubsystem climber = new ClimberSubsystem();
            });

    String expectedMessage = "not allowed! ctor must provide parameters for all dependencies";
    String actualMessage = exception.getMessage();

    // Assert
    assertTrue(actualMessage.contains(expectedMessage));
  }

  @Test
  public void itShouldStop() {

    // Assemble
    WPI_VictorSPX leftClimber = mock(WPI_VictorSPX.class);
    WPI_TalonSRX rightClimber = mock(WPI_TalonSRX.class);
    DigitalInput leftLimit = mock(DigitalInput.class);
    DigitalInput rightLimit = mock(DigitalInput.class);
    DigitalInput rightHookLimit = mock(DigitalInput.class);
    DigitalInput leftHookLimit = mock(DigitalInput.class);

    ClimberSubsystem climber =
        new ClimberSubsystem(
            leftClimber, rightClimber, leftLimit, rightLimit, leftHookLimit, rightHookLimit);

    // Act
    climber.stop();

    // Assert
    verify(leftClimber, times(1)).set(0);
    verify(rightClimber, times(1)).set(0);
  }

  @Test
  public void itShouldReferenceDevice7ForLeftClimber() {
    assertEquals(
        7,
        Constants.LeftClimbCAN,
        "Tell electrical that LeftClimbCAN Device ID has changed and update this test.");
  }

  @Test
  public void itShouldReferenceDevice7ForRightClimber() {
    assertEquals(
        12,
        Constants.RightClimbCAN,
        "Tell electrical that RightClimbCAN Device ID has changed and update this test.");
  }

  @Test
  public void itShouldReferenceChannel6ForLeftLimitDIO() {
    assertEquals(
        6,
        Constants.Climber_LeftLimitDIO,
        "Tell electrical that LeftLimitDIO Channel ID has changed and update this test.");
  }

  @Test
  public void itShouldReferenceChannel7ForRightLimitDIO() {
    assertEquals(
        7,
        Constants.Climber_RightLimitDIO,
        "Tell electrical that RightLimitDIO Channel ID has changed and update this test.");
  }

  @Test
  public void itShouldReferenceChannel9ForLeftHookDIO() {
    assertEquals(
        9,
        Constants.Hook_LeftDIO,
        "Tell electrical that Hook_LeftDIO Channel ID has changed and update this test.");
  }

  @Test
  public void itShouldReferenceChannel8ForRightHookDIO() {
    assertEquals(
        8,
        Constants.Hook_RightDIO,
        "Tell electrical that Hook_RightDIO Channel ID has changed and update this test.");
  }
}
