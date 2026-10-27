package autopilot.core;

import autopilot.core.GearBox.Gear;
import autopilot.exception.IllegalGearChangeException;
import autopilot.sensor.SpeedSensor;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
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
    void changeTo_ParkToReverseAboveSpeedLimit_ThrowsIllegalGearChangeException() {
        speedSensor.setSpeed(SPEED_ABOVE_LIMIT_KMH);

        assertThrows(IllegalGearChangeException.class, () -> gearBox.changeTo(Gear.R));
    }

    @Test
    void changeTo_ParkToDriveDirectly_ThrowsIllegalGearChangeException() {
        speedSensor.setSpeed(ZERO_SPEED_KMH);

        assertThrows(IllegalGearChangeException.class, () -> gearBox.changeTo(Gear.D));
    }

    @Test
    void changeTo_ParkToNeutralDirectly_ThrowsIllegalGearChangeException() {
        speedSensor.setSpeed(ZERO_SPEED_KMH);

        assertThrows(IllegalGearChangeException.class, () -> gearBox.changeTo(Gear.N));
    }

    @Test
    void changeTo_ReverseToDriveAtZeroSpeed_TransitionsSuccessfully() {
        transitionTo(Gear.R, ZERO_SPEED_KMH);

        gearBox.changeTo(Gear.D);

        assertEquals(Gear.D, gearBox.getCurrentGear());
    }

    @Test
    void changeTo_ReverseToDriveWhileMoving_ThrowsIllegalGearChangeException() {
        transitionTo(Gear.R, ZERO_SPEED_KMH);
        speedSensor.setSpeed(MOVING_SPEED_KMH);

        assertThrows(IllegalGearChangeException.class, () -> gearBox.changeTo(Gear.D));
    }

    @Test
    void changeTo_DriveToReverseAtZeroSpeed_TransitionsSuccessfully() {
        transitionTo(Gear.D, ZERO_SPEED_KMH);

        gearBox.changeTo(Gear.R);

        assertEquals(Gear.R, gearBox.getCurrentGear());
    }

    @Test
    void changeTo_DriveToReverseWhileMoving_ThrowsIllegalGearChangeException() {
        transitionTo(Gear.D, ZERO_SPEED_KMH);
        speedSensor.setSpeed(MOVING_SPEED_KMH);

        assertThrows(IllegalGearChangeException.class, () -> gearBox.changeTo(Gear.R));
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

    @Test
    void changeTo_NegativeSpeed_ThrowsIllegalArgumentException() {
        speedSensor.setSpeed(NEGATIVE_SPEED_KMH);

        assertThrows(IllegalArgumentException.class, () -> gearBox.changeTo(Gear.R));
    }

    @Test
    void changeTo_NaNSpeed_ThrowsIllegalArgumentException() {
        speedSensor.setSpeed(Double.NaN);

        assertThrows(IllegalArgumentException.class, () -> gearBox.changeTo(Gear.R));
    }

    @Test
    void changeTo_InfiniteSpeed_ThrowsIllegalArgumentException() {
        speedSensor.setSpeed(Double.POSITIVE_INFINITY);

        assertThrows(IllegalArgumentException.class, () -> gearBox.changeTo(Gear.R));
    }

    @Test
    void changeTo_IllegalTransitionAttempted_PreservesOriginalGear() {
        speedSensor.setSpeed(SPEED_ABOVE_LIMIT_KMH);

        assertThrows(IllegalGearChangeException.class, () -> gearBox.changeTo(Gear.R));
        assertEquals(Gear.P, gearBox.getCurrentGear());
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
