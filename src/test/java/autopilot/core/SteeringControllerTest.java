package autopilot.core;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.ValueSource;

import static org.junit.jupiter.api.Assertions.*;

class SteeringControllerTest {

    private static final double ZERO_ANGLE_DEGREES = 0.0;
    private static final double CITY_SPEED_KMH = 50.0;
    private static final double MAX_CHANGE_PER_TICK_DEGREES = 60.0;

    private SteeringController steeringController;

    @BeforeEach
    void setUp() {
        steeringController = new SteeringController();
    }

    @ParameterizedTest
    @CsvSource({
            "30.0,   50.0,  30.0,  30.0",
            "-45.0,  50.0, -45.0, -45.0",
            "60.0,   50.0,  60.0,  60.0",
            "90.0,   50.0,  90.0,  60.0",
            "-90.0,  50.0, -90.0, -60.0",
            "500.0,  50.0, 450.0,  60.0",
            "-500.0, 50.0, -450.0, -60.0",
            "90.0,  120.0,  90.0,  60.0",
            "150.0, 120.0, 150.0,  60.0",
            "100.0, 120.1,  90.0,  60.0",
            "-120.0, 130.0, -90.0, -60.0",
            "45.0,  130.0,  45.0,  45.0"
    })
    void update_VariousAnglesAndSpeeds_ClampsAndSlewsCorrectly(
            double requestedAngle, double speedKmh, double expectedTargetAngle, double expectedCurrentAngle) {
        double resultAngle = steeringController.update(requestedAngle, speedKmh);

        assertEquals(expectedTargetAngle, steeringController.getTargetAngleDegrees());
        assertEquals(expectedCurrentAngle, resultAngle);
        assertEquals(expectedCurrentAngle, steeringController.getCurrentAngleDegrees());
    }

    @ParameterizedTest
    @ValueSource(doubles = {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void update_InvalidAngle_ThrowsIllegalArgumentException(double invalidAngle) {
        assertThrows(IllegalArgumentException.class, () -> steeringController.update(invalidAngle, CITY_SPEED_KMH));
    }

    @ParameterizedTest
    @ValueSource(doubles = {-10.0, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void update_InvalidSpeed_ThrowsIllegalArgumentException(double invalidSpeed) {
        assertThrows(IllegalArgumentException.class, () ->
                steeringController.update(ZERO_ANGLE_DEGREES, invalidSpeed));
    }

    @Test
    void update_MultiTickConvergence_ReachesTargetAngleAcrossMultipleTicks() {
        double requestedAngle = 100.0;

        double tick1 = steeringController.update(requestedAngle, CITY_SPEED_KMH);
        assertEquals(MAX_CHANGE_PER_TICK_DEGREES, tick1);

        double tick2 = steeringController.update(requestedAngle, CITY_SPEED_KMH);
        assertEquals(requestedAngle, tick2);
    }

    @ParameterizedTest
    @ValueSource(doubles = {45.0, -45.0, 60.0, -60.0})
    void selfCenter_WithinSlewLimit_ReachesZeroInSingleTick(double initialAngle) {
        steeringController.update(initialAngle, CITY_SPEED_KMH);

        double centeredAngle = steeringController.selfCenter();

        assertEquals(ZERO_ANGLE_DEGREES, centeredAngle);
        assertEquals(ZERO_ANGLE_DEGREES, steeringController.getCurrentAngleDegrees());
    }

    @ParameterizedTest
    @CsvSource({
            "100.0,   40.0",
            "-100.0, -40.0"
    })
    void selfCenter_BeyondSlewLimit_MovesTowardZeroByMaxChangePerTick(
            double initialAngle, double expectedFirstStep) {
        steeringController.update(initialAngle, CITY_SPEED_KMH);
        steeringController.update(initialAngle, CITY_SPEED_KMH);

        double step1 = steeringController.selfCenter();
        assertEquals(expectedFirstStep, step1);

        double step2 = steeringController.selfCenter();
        assertEquals(ZERO_ANGLE_DEGREES, step2);
    }

    @Test
    void selfCenter_AlreadyCentered_RemainsAtZero() {
        double centeredAngle = steeringController.selfCenter();

        assertEquals(ZERO_ANGLE_DEGREES, centeredAngle);
    }
}
