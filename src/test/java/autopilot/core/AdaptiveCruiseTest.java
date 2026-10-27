package autopilot.core;

import autopilot.core.AdaptiveCruise.Decision;
import autopilot.core.AdaptiveCruise.Mode;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.CsvFileSource;
import org.junit.jupiter.params.provider.MethodSource;
import org.junit.jupiter.params.provider.ValueSource;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.*;

class AdaptiveCruiseTest {

    private static final double TARGET_SPEED_KMH = 80.0;
    private static final double ZERO_SPEED_KMH = 0.0;
    private static final double ZERO_DISTANCE_METERS = 0.0;
    private static final double NEGATIVE_VALUE = -5.0;

    private static final double HYSTERESIS_PRESSURE = 10.0;
    private static final double EMERGENCY_PRESSURE = 100.0;

    private AdaptiveCruise adaptiveCruise;

    @BeforeEach
    void setUp() {
        adaptiveCruise = new AdaptiveCruise(TARGET_SPEED_KMH);
    }

    @ParameterizedTest
    @CsvFileSource(resources = "/cruise_safe_distance_data.csv", numLinesToSkip = 1)
    void safeDistanceFor_VariousSpeedsFromCsv_CalculatesExpectedDistance(
            double speedKmh, double expectedDistanceMeters) {
        double safeDistance = adaptiveCruise.safeDistanceFor(speedKmh);

        assertEquals(expectedDistanceMeters, safeDistance);
    }

    @ParameterizedTest
    @MethodSource("provideUpdateDecisions")
    void update_VariousConditions_ReturnsExpectedCruiseDecision(
            double speedKmh, double distanceMeters, Mode expectedMode, double expectedAmount) {
        Decision decision = adaptiveCruise.update(speedKmh, distanceMeters);

        assertEquals(expectedMode, decision.mode());
        assertEquals(expectedAmount, decision.amount(), 1e-6);
    }

    private static Stream<Arguments> provideUpdateDecisions() {
        // With TARGET_SPEED_KMH = 80.0:
        // At speed = 80.0: safeDistance = 40.0, emergencyDistance = 20.0, releaseDistance = 48.0
        return Stream.of(
                // 1. Emergency braking: distance <= emergencyDistance (20m)
                Arguments.of(80.0, 15.0, Mode.EMERGENCY_BRAKE, EMERGENCY_PRESSURE),
                Arguments.of(80.0, 20.0, Mode.EMERGENCY_BRAKE, EMERGENCY_PRESSURE),

                // 2. Proportional braking: emergencyDistance < distance < safeDistance (40m)
                // (40 - 30) / 40 * 100 = 25.0%
                Arguments.of(80.0, 30.0, Mode.BRAKE, 25.0),

                // 3. At boundary distance == safeDistance (neither < safe nor > safe)
                Arguments.of(80.0, 40.0, Mode.HOLD, 0.0),

                // 4. Acceleration capped at MAX_ACCELERATION_STEP_KMH (5.0 km/h)
                Arguments.of(60.0, 50.0, Mode.ACCELERATE, 5.0),

                // 5. Acceleration below MAX_ACCELERATION_STEP_KMH (target 80 - current 78 = 2.0 km/h)
                Arguments.of(78.0, 50.0, Mode.ACCELERATE, 2.0),

                // 6. Holding speed when current speed == target speed and distance is safe
                Arguments.of(80.0, 50.0, Mode.HOLD, 0.0),

                // 7. Holding speed when current speed > target speed
                Arguments.of(85.0, 50.0, Mode.HOLD, 0.0)
        );
    }

    @ParameterizedTest
    @ValueSource(doubles = {-5.0, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void safeDistanceFor_InvalidSpeed_ThrowsIllegalArgumentException(double invalidSpeed) {
        assertThrows(IllegalArgumentException.class, () -> adaptiveCruise.safeDistanceFor(invalidSpeed));
    }

    @ParameterizedTest
    @ValueSource(doubles = {-5.0, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void update_InvalidSpeed_ThrowsIllegalArgumentException(double invalidSpeed) {
        assertThrows(IllegalArgumentException.class, () -> adaptiveCruise.update(invalidSpeed, 50.0));
    }

    @ParameterizedTest
    @ValueSource(doubles = {-5.0, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void update_InvalidDistance_ThrowsIllegalArgumentException(double invalidDistance) {
        assertThrows(IllegalArgumentException.class, () -> adaptiveCruise.update(TARGET_SPEED_KMH, invalidDistance));
    }

    @Test
    void update_ZeroSpeedAndZeroDistance_TriggersEmergencyBrake() {
        Decision decision = adaptiveCruise.update(ZERO_SPEED_KMH, ZERO_DISTANCE_METERS);

        assertEquals(Mode.EMERGENCY_BRAKE, decision.mode());
        assertEquals(EMERGENCY_PRESSURE, decision.amount());
    }

    @Test
    void update_WithinHysteresisReleaseZone_MaintainsHysteresisBraking() {
        // Step 1: Engage braking at speed 80 km/h with distance 30m (< safeDistance 40m)
        adaptiveCruise.update(80.0, 30.0);

        // Step 2: Distance increases to 44m (> safeDistance 40m, but < releaseDistance 48m)
        Decision decision = adaptiveCruise.update(80.0, 44.0);

        assertEquals(Mode.BRAKE, decision.mode());
        assertEquals(HYSTERESIS_PRESSURE, decision.amount());
    }

    @Test
    void update_ExceedingReleaseDistance_ReleasesBrakeAndAccelerates() {
        // Step 1: Engage braking at speed 70 km/h with distance 25m (< safeDistance 35m)
        adaptiveCruise.update(70.0, 25.0);

        // Step 2: Distance clears past releaseDistance (1.2 * 35 = 42m)
        Decision decision = adaptiveCruise.update(70.0, 45.0);

        assertEquals(Mode.ACCELERATE, decision.mode());
        assertEquals(5.0, decision.amount());
    }
}
