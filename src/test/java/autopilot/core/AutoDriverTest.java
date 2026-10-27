package autopilot.core;

import autopilot.core.GearBox.Gear;
import autopilot.exception.SensorFailureException;
import autopilot.sensor.LaneSensor;
import autopilot.sensor.ObstacleSensor;
import autopilot.sensor.SpeedSensor;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.*;

class AutoDriverTest {

    // =========================================================================
    // REFACTORING 1: Elimination of Magic Numbers and Magic Strings
    // REASON: As required by docs/SWT-05-CA1.pdf ("وجود magic number و magic string")
    // and Slide 10 of "05-Principles-of-Effective-Test-Design.md", hardcoded
    // literals (80.0, 100.0, 0.0, 30.0, "Speed sensor failed") were replaced
    // with descriptive, self-documenting constants to communicate domain intent.
    // =========================================================================
    private static final double TARGET_SPEED_KMH = 80.0;
    private static final double NORMAL_SPEED_KMH = 80.0;
    private static final double CLEAR_DISTANCE_METERS = 100.0;
    private static final double CENTERED_OFFSET_METERS = 0.0;

    private static final double NO_BRAKE_PRESSURE = 0.0;
    private static final double SAFE_MODE_BRAKE_PRESSURE = 30.0;
    private static final double CENTERED_STEERING_ANGLE = 0.0;
    private static final String SENSOR_FAILURE_MESSAGE = "Sensor communication failure";

    private SpeedSensor speedSensor;
    private ObstacleSensor obstacleSensor;
    private LaneSensor laneSensor;
    private AutoDriver autoDriver;

    // =========================================================================
    // REFACTORING 2: Replacing Verbose Anonymous Classes with Lambdas
    // REASON: The original code used 8-line anonymous inner classes
    // (new SpeedSensor() { public double getSpeedKmh() { return 80.0; } })
    // for each sensor. Using standard Java lambdas (() -> NORMAL_SPEED_KMH)
    // drastically cuts boilerplate noise while keeping test fixtures clean.
    // =========================================================================
    @BeforeEach
    void setUp() {
        speedSensor = () -> NORMAL_SPEED_KMH;
        obstacleSensor = () -> CLEAR_DISTANCE_METERS;
        laneSensor = () -> CENTERED_OFFSET_METERS;

        autoDriver = new AutoDriver(
                speedSensor,
                obstacleSensor,
                laneSensor,
                TARGET_SPEED_KMH
        );
    }

    // =========================================================================
    // REFACTORING 3: Renaming 'test1' and Pruning Redundant Assertions
    // REASON:
    // 1. Name: 'test1' was uninformative and violated the required naming standard
    //    'MethodName_StateUnderTest_ExpectedBehavior' (SWT-05-CA1.pdf: "نام‌گذاری نامفهوم").
    // 2. Assertions: Original test contained excessive/unfocused assertions
    //    (assertNotNull(result), assertNotNull(result.cruiseMode())) alongside
    //    business assertions. In line with the "Verify One Condition per Test"
    //    principle (06-Principals of Test Automation.md, Slide 10), it now
    //    specifically validates that normal cruising maintains speed without braking.
    // =========================================================================
    @Test
    void tick_NormalDrivingConditions_MaintainsTargetSpeedWithoutBraking() {
        AutoDriver.DriveResult result = autoDriver.tick(Gear.D);

        assertFalse(result.safeMode());
        assertEquals(NO_BRAKE_PRESSURE, result.brakePressure());
    }

    // =========================================================================
    // REFACTORING 4: Consolidating 'testLanePositive' & 'testLaneNegative'
    // REASON:
    // 1. Both original tests had identical structure, duplicated SUT instantiation,
    //    and duplicated assertion logic ("وجود کد یا assertion تکراری").
    // 2. Used JUnit 5 @ParameterizedTest with @CsvSource to test opposing lane
    //    deviations (+1.0m drift -> steer left -45°, -1.0m drift -> steer right +45°)
    //    in a single data-driven test, eliminating code duplication (DRY).
    // 3. Renamed to 'MethodName_StateUnderTest_ExpectedBehavior'.
    // =========================================================================
    @ParameterizedTest
    @CsvSource({
            "1.0,  -45.0",  // Right drift (> 0.5m) -> steer left (negative angle)
            "-1.0,  45.0"   // Left drift (< -0.5m) -> steer right (positive angle)
    })
    void tick_SignificantLaneDeviation_AppliesOpposingSteeringCorrection(
            double laneOffsetMeters, double expectedSteeringAngle) {
        laneSensor = () -> laneOffsetMeters;
        autoDriver = new AutoDriver(speedSensor, obstacleSensor, laneSensor, TARGET_SPEED_KMH);

        AutoDriver.DriveResult result = autoDriver.tick(Gear.D);

        assertEquals(expectedSteeringAngle, result.steeringAngleDegrees());
    }

    // =========================================================================
    // REFACTORING 5: Parameterizing Sensor Failure Tests with @MethodSource
    // REASON:
    // 1. 'testSpeedSensorFailure' and 'testObstacleSensorFailure' duplicated an
    //    identical 5-assertion block checking safe mode properties.
    // 2. Using @ParameterizedTest with @MethodSource("provideFailingSensors")
    //    eliminates the duplicated code into a single reusable verification loop.
    // 3. Expands coverage to also test LaneSensor failure (which was missing in
    //    the original suite).
    // 4. Renamed from vague names to 'MethodName_StateUnderTest_ExpectedBehavior'.
    // =========================================================================
    @ParameterizedTest(name = "Sensor failure on {3} engages safe mode")
    @MethodSource("provideFailingSensors")
    void tick_SensorFailure_EngagesSafeModeWithDegradedBraking(
            SpeedSensor speed, ObstacleSensor obstacle, LaneSensor lane, String failingSensorName) {
        autoDriver = new AutoDriver(speed, obstacle, lane, TARGET_SPEED_KMH);

        AutoDriver.DriveResult result = autoDriver.tick(Gear.D);

        assertTrue(result.safeMode());
        assertEquals(SAFE_MODE_BRAKE_PRESSURE, result.brakePressure());
        assertEquals(CENTERED_STEERING_ANGLE, result.steeringAngleDegrees());
        assertEquals(AdaptiveCruise.Mode.BRAKE, result.cruiseMode());
        assertFalse(result.warningActive());
    }

    private static Stream<Arguments> provideFailingSensors() {
        SpeedSensor normalSpeed = () -> NORMAL_SPEED_KMH;
        ObstacleSensor normalObstacle = () -> CLEAR_DISTANCE_METERS;
        LaneSensor normalLane = () -> CENTERED_OFFSET_METERS;

        SpeedSensor failingSpeed = () -> {
            throw new SensorFailureException(SENSOR_FAILURE_MESSAGE);
        };
        ObstacleSensor failingObstacle = () -> {
            throw new SensorFailureException(SENSOR_FAILURE_MESSAGE);
        };
        LaneSensor failingLane = () -> {
            throw new SensorFailureException(SENSOR_FAILURE_MESSAGE);
        };

        return Stream.of(
                Arguments.of(failingSpeed, normalObstacle, normalLane, "SpeedSensor"),
                Arguments.of(normalSpeed, failingObstacle, normalLane, "ObstacleSensor"),
                Arguments.of(normalSpeed, normalObstacle, failingLane, "LaneSensor")
        );
    }
}
