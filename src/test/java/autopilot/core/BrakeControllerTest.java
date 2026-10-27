package autopilot.core;

import autopilot.core.GearBox.Gear;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

import static org.junit.jupiter.api.Assertions.*;

class BrakeControllerTest {

    private static final double ZERO_SPEED_KMH = 0.0;
    private static final double CITY_SPEED_KMH = 50.0;
    private static final double ABS_SPEED_THRESHOLD_KMH = 30.0;
    private static final double ABS_ACTIVE_SPEED_KMH = 60.0;
    private static final double BELOW_ABS_SPEED_KMH = 20.0;

    private static final double NORMAL_PRESSURE = 50.0;
    private static final double ABS_PRESSURE_THRESHOLD = 80.0;
    private static final double HIGH_PRESSURE = 95.0;
    private static final double EXCESSIVE_PRESSURE = 150.0;
    private static final double MAX_PRESSURE = 100.0;
    private static final double ABS_HIGH_PULSE = 90.0;
    private static final double ABS_LOW_PULSE = 60.0;
    private static final double NEGATIVE_VALUE = -10.0;

    private BrakeController brakeController;

    @BeforeEach
    void setUp() {
        brakeController = new BrakeController();
    }

    @ParameterizedTest
    @EnumSource(value = Gear.class, names = {"R", "N", "D"})
    void emergencyBrake_AllowedGears_AppliesMaximumPressureAndSetsWarning(Gear gear) {
        brakeController.emergencyBrake(gear);

        assertEquals(MAX_PRESSURE, brakeController.getPressure());
        assertTrue(brakeController.isWarningActive());
    }

    @Test
    void emergencyBrake_ParkGear_ThrowsIllegalStateException() {
        assertThrows(IllegalStateException.class, () -> brakeController.emergencyBrake(Gear.P));
    }

    @Test
    void emergencyBrake_NullGear_ThrowsNullPointerException() {
        assertThrows(NullPointerException.class, () -> brakeController.emergencyBrake(null));
    }

    @Test
    void apply_ParkGear_ThrowsIllegalStateException() {
        assertThrows(IllegalStateException.class, () ->
                brakeController.apply(NORMAL_PRESSURE, CITY_SPEED_KMH, Gear.P));
    }

    @Test
    void apply_NullGear_ThrowsNullPointerException() {
        assertThrows(NullPointerException.class, () ->
                brakeController.apply(NORMAL_PRESSURE, CITY_SPEED_KMH, null));
    }

    @Test
    void apply_NegativePressure_ThrowsIllegalArgumentException() {
        assertThrows(IllegalArgumentException.class, () ->
                brakeController.apply(NEGATIVE_VALUE, CITY_SPEED_KMH, Gear.D));
    }

    @Test
    void apply_NaNPressure_ThrowsIllegalArgumentException() {
        assertThrows(IllegalArgumentException.class, () ->
                brakeController.apply(Double.NaN, CITY_SPEED_KMH, Gear.D));
    }

    @Test
    void apply_InfinitePressure_ThrowsIllegalArgumentException() {
        assertThrows(IllegalArgumentException.class, () ->
                brakeController.apply(Double.POSITIVE_INFINITY, CITY_SPEED_KMH, Gear.D));
    }

    @Test
    void apply_NegativeSpeed_ThrowsIllegalArgumentException() {
        assertThrows(IllegalArgumentException.class, () ->
                brakeController.apply(NORMAL_PRESSURE, NEGATIVE_VALUE, Gear.D));
    }

    @Test
    void apply_NaNSpeed_ThrowsIllegalArgumentException() {
        assertThrows(IllegalArgumentException.class, () ->
                brakeController.apply(NORMAL_PRESSURE, Double.NaN, Gear.D));
    }

    @Test
    void apply_InfiniteSpeed_ThrowsIllegalArgumentException() {
        assertThrows(IllegalArgumentException.class, () ->
                brakeController.apply(NORMAL_PRESSURE, Double.POSITIVE_INFINITY, Gear.D));
    }

    @Test
    void apply_NormalPressureBelowAbsThreshold_AppliesRequestedPressureDirectly() {
        double pressure = brakeController.apply(NORMAL_PRESSURE, CITY_SPEED_KMH, Gear.D);

        assertEquals(NORMAL_PRESSURE, pressure);
        assertEquals(NORMAL_PRESSURE, brakeController.getPressure());
        assertFalse(brakeController.isWarningActive());
    }

    @Test
    void apply_PressureExceedingMaxLimitAtLowSpeed_CapsAtMaxPressure() {
        double pressure = brakeController.apply(EXCESSIVE_PRESSURE, BELOW_ABS_SPEED_KMH, Gear.D);

        assertEquals(MAX_PRESSURE, pressure);
    }

    @Test
    void apply_AtAbsPressureBoundary_DoesNotTriggerAbs() {
        double pressure = brakeController.apply(ABS_PRESSURE_THRESHOLD, ABS_ACTIVE_SPEED_KMH, Gear.D);

        assertEquals(ABS_PRESSURE_THRESHOLD, pressure);
    }

    @Test
    void apply_AtAbsSpeedBoundary_DoesNotTriggerAbs() {
        double pressure = brakeController.apply(HIGH_PRESSURE, ABS_SPEED_THRESHOLD_KMH, Gear.D);

        assertEquals(HIGH_PRESSURE, pressure);
    }

    @Test
    void apply_ExceedingAbsThresholds_AlternatesBetweenHighAndLowPulses() {
        double pulse1 = brakeController.apply(HIGH_PRESSURE, ABS_ACTIVE_SPEED_KMH, Gear.D);
        assertEquals(ABS_HIGH_PULSE, pulse1);

        double pulse2 = brakeController.apply(HIGH_PRESSURE, ABS_ACTIVE_SPEED_KMH, Gear.D);
        assertEquals(ABS_LOW_PULSE, pulse2);

        double pulse3 = brakeController.apply(HIGH_PRESSURE, ABS_ACTIVE_SPEED_KMH, Gear.D);
        assertEquals(ABS_HIGH_PULSE, pulse3);
    }

    @Test
    void apply_AbsConditionsStop_ResetsPulseSequence() {
        // Trigger pulse 1 (high pulse)
        brakeController.apply(HIGH_PRESSURE, ABS_ACTIVE_SPEED_KMH, Gear.D);

        // Conditions drop below ABS threshold
        double intermediatePressure = brakeController.apply(NORMAL_PRESSURE, BELOW_ABS_SPEED_KMH, Gear.D);
        assertEquals(NORMAL_PRESSURE, intermediatePressure);

        // Next ABS trigger should start afresh with high pulse
        double nextPulse = brakeController.apply(HIGH_PRESSURE, ABS_ACTIVE_SPEED_KMH, Gear.D);
        assertEquals(ABS_HIGH_PULSE, nextPulse);
    }
}
