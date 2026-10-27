package autopilot.core;

import autopilot.core.GearBox.Gear;
import autopilot.exception.IllegalGearChangeException;
import autopilot.sensor.SpeedSensor;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.params.provider.ValueSource;

import static org.junit.jupiter.api.Assertions.*;

class GearBoxTest {

    private static final double ZERO_SPEED_KMH = 0.0;
    private static final double REVERSE_SPEED_LIMIT_KMH = 5.0;
    private static final double SPEED_ABOVE_LIMIT_KMH = 10.0;
    private static final double HIGHWAY_SPEED_KMH = 80.0;
    private static final double MOVING_SPEED_KMH = 2.0;
    private static final double NEGATIVE_SPEED_KMH = -1.0;

    private MutableSpeedSensor speedSensor;
    private GearBox gearBox;

    @BeforeEach
    void setUp() {
        speedSensor = new MutableSpeedSensor(ZERO_SPEED_KMH);
        gearBox = new GearBox(speedSensor);
    }

    @Test
    void changeTo_TargetGearEqualsCurrentGear_RemainsInSameGearWithoutQueryingSpeed() {
        speedSensor.setSpeed(NEGATIVE_SPEED_KMH);

        gearBox.changeTo(Gear.P);

        assertEquals(Gear.P, gearBox.getCurrentGear());
    }

    @Test
    void changeTo_NullTargetGear_ThrowsNullPointerException() {
        assertThrows(NullPointerException.class, () -> gearBox.changeTo(null));
    }

    @ParameterizedTest
    @ValueSource(doubles = {0.0, 1.5, 3.0, 4.9, 4.99})
    void changeTo_ParkToReverseUnderSpeedLimit_TransitionsSuccessfully(double validCrawlingSpeed) {
        speedSensor.setSpeed(validCrawlingSpeed);

        gearBox.changeTo(Gear.R);

        assertEquals(Gear.R, gearBox.getCurrentGear());
    }

    @Test
    void changeTo_ParkToReverseAtExactSpeedLimit_ThrowsIllegalGearChangeException() {
        speedSensor.setSpeed(REVERSE_SPEED_LIMIT_KMH);

        assertThrows(IllegalGearChangeException.class, () -> gearBox.changeTo(Gear.R));
    }

    @Test
    void changeTo_ParkToReverseAboveSpeedLimit_ThrowsExceptionAndPreservesCurrentGear() {
        speedSensor.setSpeed(SPEED_ABOVE_LIMIT_KMH);

        assertThrows(IllegalGearChangeException.class, () -> gearBox.changeTo(Gear.R));
        assertEquals(Gear.P, gearBox.getCurrentGear());
    }

    @ParameterizedTest
    @EnumSource(value = Gear.class, names = {"D", "N"})
    void changeTo_ParkToDisallowedGearDirectly_ThrowsIllegalGearChangeException(Gear disallowedTargetGear) {
        speedSensor.setSpeed(ZERO_SPEED_KMH);

        assertThrows(IllegalGearChangeException.class, () -> gearBox.changeTo(disallowedTargetGear));
    }

    @ParameterizedTest
    @CsvSource({
            "R, D",
            "D, R"
    })
    void changeTo_BetweenReverseAndDriveAtZeroSpeed_TransitionsSuccessfully(Gear fromGear, Gear toGear) {
        transitionTo(fromGear, ZERO_SPEED_KMH);

        gearBox.changeTo(toGear);

        assertEquals(toGear, gearBox.getCurrentGear());
    }

    @ParameterizedTest
    @CsvSource({
            "R, D",
            "D, R"
    })
    void changeTo_BetweenReverseAndDriveWhileMoving_ThrowsIllegalGearChangeException(
            Gear fromGear, Gear toGear) {
        transitionTo(fromGear, ZERO_SPEED_KMH);
        speedSensor.setSpeed(MOVING_SPEED_KMH);

        assertThrows(IllegalGearChangeException.class, () -> gearBox.changeTo(toGear));
    }

    @Test
    void changeTo_ReverseToNeutralWhileMoving_TransitionsSuccessfully() {
        transitionTo(Gear.R, ZERO_SPEED_KMH);
        speedSensor.setSpeed(HIGHWAY_SPEED_KMH);

        gearBox.changeTo(Gear.N);

        assertEquals(Gear.N, gearBox.getCurrentGear());
    }

    @Test
    void changeTo_NeutralToDriveWhileMoving_TransitionsSuccessfully() {
        transitionTo(Gear.N, ZERO_SPEED_KMH);
        speedSensor.setSpeed(HIGHWAY_SPEED_KMH);

        gearBox.changeTo(Gear.D);

        assertEquals(Gear.D, gearBox.getCurrentGear());
    }

    @Test
    void changeTo_DriveToNeutral_ThrowsIllegalGearChangeException() {
        transitionTo(Gear.D, ZERO_SPEED_KMH);

        assertThrows(IllegalGearChangeException.class, () -> gearBox.changeTo(Gear.N));
    }

    @ParameterizedTest
    @ValueSource(doubles = {-1.0, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void changeTo_InvalidSpeedValue_ThrowsIllegalArgumentException(double invalidSpeed) {
        speedSensor.setSpeed(invalidSpeed);

        assertThrows(IllegalArgumentException.class, () -> gearBox.changeTo(Gear.R));
    }

    private void transitionTo(Gear target, double speed) {
        if (target == Gear.R) {
            speedSensor.setSpeed(ZERO_SPEED_KMH);
            gearBox.changeTo(Gear.R);
        } else if (target == Gear.N) {
            transitionTo(Gear.R, ZERO_SPEED_KMH);
            gearBox.changeTo(Gear.N);
        } else if (target == Gear.D) {
            transitionTo(Gear.R, ZERO_SPEED_KMH);
            gearBox.changeTo(Gear.D);
        }
        speedSensor.setSpeed(speed);
    }

    private static final class MutableSpeedSensor implements SpeedSensor {
        private double speed;

        MutableSpeedSensor(double initialSpeed) {
            this.speed = initialSpeed;
        }

        void setSpeed(double newSpeed) {
            this.speed = newSpeed;
        }

        @Override
        public double getSpeedKmh() {
            return speed;
        }
    }
}
