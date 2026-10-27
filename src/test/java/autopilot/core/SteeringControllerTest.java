package autopilot.core;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

import static org.junit.jupiter.api.Assertions.*;

class SteeringControllerTest {

    private static final double ZERO_ANGLE_DEGREES = 0.0;
    private static final double CITY_SPEED_KMH = 50.0;
    private static final double HIGHWAY_SPEED_KMH = 130.0;
    private static final double THRESHOLD_SPEED_KMH = 120.0;
    private static final double NEGATIVE_SPEED_KMH = -10.0;
    private static final double MAX_CHANGE_PER_TICK_DEGREES = 60.0;

    private SteeringController steeringController;

    @BeforeEach
    void setUp() {
        steeringController = new SteeringController();
    }

    @ParameterizedTest
    @CsvSource({
            // requestedAngle, speedKmh, expectedTargetAngle, expectedCurrentAngle
            "30.0,   50.0,  30.0,  30.0",   // Within slew limit and normal range
            "-45.0,  50.0, -45.0, -45.0",   // Negative angle within slew limit
            "60.0,   50.0,  60.0,  60.0",   // Exactly at slew rate boundary
            "90.0,   50.0,  90.0,  60.0",   // Exceeds slew rate: moves by max step
            "-90.0,  50.0, -90.0, -60.0",   // Negative exceeding slew rate
            "500.0,  50.0, 450.0,  60.0",   // Exceeds physical limit (450°), slews by 60°
            "-500.0, 50.0, -450.0, -60.0",  // Exceeds negative physical limit (-450°)
            "90.0,  120.0,  90.0,  60.0",   // At speed threshold boundary (120.0 km/h: physical limit holds)
            "150.0, 120.0, 150.0,  60.0",   // At speed threshold 120.0 km/h: not clamped to 90°
            "100.0, 120.1,  90.0,  60.0",   // Above threshold speed (120.1 km/h): clamped to 90°
            "-120.0, 130.0, -90.0, -60.0",  // Above threshold speed: clamped to -90°
            "45.0,  130.0,  45.0,  45.0"    // High speed within 90° limit and within slew
    })
    void update_VariousAnglesAndSpeeds_ClampsAndSlewsCorrectly(
            double requestedAngle, double speedKmh, double expectedTargetAngle, double expectedCurrentAngle) {
        double resultAngle = steeringController.update(requestedAngle, speedKmh);

        assertEquals(expectedTargetAngle, steeringController.getTargetAngleDegrees());
        assertEquals(expectedCurrentAngle, resultAngle);
        assertEquals(expectedCurrentAngle, steeringController.getCurrentAngleDegrees());
    }

    @Test
    void update_NaNAngle_ThrowsIllegalArgumentException() {
        assertThrows(IllegalArgumentException.class, () -> steeringController.update(Double.NaN, CITY_SPEED_KMH));
    }

    @Test
    void update_InfiniteAngle_ThrowsIllegalArgumentException() {
        assertThrows(IllegalArgumentException.class, () ->
                steeringController.update(Double.POSITIVE_INFINITY, CITY_SPEED_KMH));
    }

    @Test
    void update_NegativeSpeed_ThrowsIllegalArgumentException() {
        assertThrows(IllegalArgumentException.class, () ->
                steeringController.update(ZERO_ANGLE_DEGREES, NEGATIVE_SPEED_KMH));
    }

    @Test
    void update_NaNSpeed_ThrowsIllegalArgumentException() {
        assertThrows(IllegalArgumentException.class, () ->
                steeringController.update(ZERO_ANGLE_DEGREES, Double.NaN));
    }

    @Test
    void update_InfiniteSpeed_ThrowsIllegalArgumentException() {
        assertThrows(IllegalArgumentException.class, () ->
                steeringController.update(ZERO_ANGLE_DEGREES, Double.POSITIVE_INFINITY));
    }

    @Test
    void update_MultiTickConvergence_ReachesTargetAngleAcrossMultipleTicks() {
        double requestedAngle = 100.0;

        double tick1 = steeringController.update(requestedAngle, CITY_SPEED_KMH);
        assertEquals(MAX_CHANGE_PER_TICK_DEGREES, tick1);

        double tick2 = steeringController.update(requestedAngle, CITY_SPEED_KMH);
        assertEquals(requestedAngle, tick2);
    }

    @Test
    void selfCenter_WithinSlewLimit_ReachesZeroInSingleTick() {
        double initialAngle = 45.0;
        steeringController.update(initialAngle, CITY_SPEED_KMH);

        double centeredAngle = steeringController.selfCenter();

        assertEquals(ZERO_ANGLE_DEGREES, centeredAngle);
        assertEquals(ZERO_ANGLE_DEGREES, steeringController.getCurrentAngleDegrees());
    }

    @Test
    void selfCenter_NegativeWithinSlewLimit_ReachesZeroInSingleTick() {
        double initialAngle = -45.0;
        steeringController.update(initialAngle, CITY_SPEED_KMH);

        double centeredAngle = steeringController.selfCenter();

        assertEquals(ZERO_ANGLE_DEGREES, centeredAngle);
    }

    @Test
    void selfCenter_BeyondSlewLimit_MovesTowardZeroByMaxChangePerTick() {
        double initialAngle = 100.0;
        steeringController.update(initialAngle, CITY_SPEED_KMH);
        steeringController.update(initialAngle, CITY_SPEED_KMH); // reaches 100.0

        double step1 = steeringController.selfCenter();
        assertEquals(40.0, step1);

        double step2 = steeringController.selfCenter();
        assertEquals(ZERO_ANGLE_DEGREES, step2);
    }

    @Test
    void selfCenter_NegativeBeyondSlewLimit_MovesTowardZeroByMaxChangePerTick() {
        double initialAngle = -100.0;
        steeringController.update(initialAngle, CITY_SPEED_KMH);
        steeringController.update(initialAngle, CITY_SPEED_KMH); // reaches -100.0

        double step1 = steeringController.selfCenter();
        assertEquals(-40.0, step1);

        double step2 = steeringController.selfCenter();
        assertEquals(ZERO_ANGLE_DEGREES, step2);
    }

    @Test
    void selfCenter_AlreadyCentered_RemainsAtZero() {
        double centeredAngle = steeringController.selfCenter();

        assertEquals(ZERO_ANGLE_DEGREES, centeredAngle);
    }
}
