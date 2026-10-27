package autopilot.core;

import autopilot.core.GearBox.Gear;
import autopilot.exception.SensorFailureException;
import autopilot.sensor.LaneSensor;
import autopilot.sensor.ObstacleSensor;
import autopilot.sensor.SpeedSensor;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

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

    @Test
    void tick_SpeedSensorFailure_EngagesSafeModeWithDegradedBraking() {
        speedSensor = () -> {
            throw new SensorFailureException(SENSOR_FAILURE_MESSAGE);
        };
        autoDriver = new AutoDriver(speedSensor, obstacleSensor, laneSensor, TARGET_SPEED_KMH);

        AutoDriver.DriveResult result = autoDriver.tick(Gear.D);

        assertTrue(result.safeMode());
        assertEquals(SAFE_MODE_BRAKE_PRESSURE, result.brakePressure());
        assertEquals(CENTERED_STEERING_ANGLE, result.steeringAngleDegrees());
        assertEquals(AdaptiveCruise.Mode.BRAKE, result.cruiseMode());
        assertFalse(result.warningActive());
    }

    @Test
    void tick_ObstacleSensorFailure_EngagesSafeModeWithDegradedBraking() {
        obstacleSensor = () -> {
            throw new SensorFailureException(SENSOR_FAILURE_MESSAGE);
        };
        autoDriver = new AutoDriver(speedSensor, obstacleSensor, laneSensor, TARGET_SPEED_KMH);

        AutoDriver.DriveResult result = autoDriver.tick(Gear.D);

        assertTrue(result.safeMode());
        assertEquals(SAFE_MODE_BRAKE_PRESSURE, result.brakePressure());
        assertEquals(CENTERED_STEERING_ANGLE, result.steeringAngleDegrees());
        assertEquals(AdaptiveCruise.Mode.BRAKE, result.cruiseMode());
        assertFalse(result.warningActive());
    }
}
