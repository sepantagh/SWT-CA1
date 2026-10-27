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

    @Test
    void tick_NormalDrivingConditions_MaintainsTargetSpeedWithoutBraking() {
        AutoDriver.DriveResult result = autoDriver.tick(Gear.D);

        assertFalse(result.safeMode());
        assertEquals(NO_BRAKE_PRESSURE, result.brakePressure());
    }

    @ParameterizedTest
    @CsvSource({
            "1.0,  -45.0",
            "-1.0,  45.0"
    })
    void tick_SignificantLaneDeviation_AppliesOpposingSteeringCorrection(
            double laneOffsetMeters, double expectedSteeringAngle) {
        laneSensor = () -> laneOffsetMeters;
        autoDriver = new AutoDriver(speedSensor, obstacleSensor, laneSensor, TARGET_SPEED_KMH);

        AutoDriver.DriveResult result = autoDriver.tick(Gear.D);

        assertEquals(expectedSteeringAngle, result.steeringAngleDegrees());
    }

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
