package frc.robot;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

import java.util.function.DoubleSupplier;

public abstract class FuntionalPIDSource implements PIDSource {
    @Override public void setPIDSourceType(PIDSourceType pidSource) {}
    @Override public double pidGet() {return pidGetFunction.getAsDouble();}
    private DoubleSupplier pidGetFunction;
    private FuntionalPIDSource(DoubleSupplier pidGetFunction) {
        this.pidGetFunction = pidGetFunction;
    }

    public static class Displacement extends FuntionalPIDSource {
        @Override public PIDSourceType getPIDSourceType() {return PIDSourceType.kDisplacement;}
        public Displacement(DoubleSupplier pidGetFunction) {super(pidGetFunction);}
    }

    public static class Rate extends FuntionalPIDSource {
        @Override public PIDSourceType getPIDSourceType() {return PIDSourceType.kRate;}
        public Rate(DoubleSupplier pidGetFunction) {super(pidGetFunction);}
    }
}
