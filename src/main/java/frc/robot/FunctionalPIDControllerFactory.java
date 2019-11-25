package frc.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class FunctionalPIDControllerFactory {
      private FunctionalPIDControllerFactory(){}

      public static PIDController build(
              double kP,
              double kI,
              double kD,
              double kF,
              DoubleSupplier pidInput,
              DoubleConsumer pidOutput,
              PIDSourceType pidSourceType,
              double period
      ) {
            return new PIDController(
                    kP,kI,kD,kF,
                    getPIDSourceWrapper(pidSourceType,pidInput),
                    pidOutput::accept,
                    period
            );
      }
      public static PIDController build(
              double kP,
              double kI,
              double kD,
              DoubleSupplier pidInput,
              DoubleConsumer pidOutput,
              PIDSourceType pidSourceType,
              double period
      ) {
            return new PIDController(
                    kP,kI,kD,
                    getPIDSourceWrapper(pidSourceType,pidInput),
                    pidOutput::accept,
                    period
            );
      }
      public static PIDController build(
              double kP,
              double kI,
              double kD,
              double kF,
              DoubleSupplier pidInput,
              DoubleConsumer pidOutput,
              PIDSourceType pidSourceType
      ) {
            return new PIDController(
                    kP,kI,kD,kF,
                    getPIDSourceWrapper(pidSourceType,pidInput),
                    pidOutput::accept
            );
      }
      public static PIDController build(
              double kP,
              double kI,
              double kD,
              DoubleSupplier pidInput,
              DoubleConsumer pidOutput,
              PIDSourceType pidSourceType
      ) {
            return new PIDController(
                    kP,kI,kD,
                    getPIDSourceWrapper(pidSourceType,pidInput),
                    pidOutput::accept
            );
      }

      private static FuntionalPIDSource getPIDSourceWrapper(
              PIDSourceType type,
              DoubleSupplier func
      ) {

            FuntionalPIDSource pidInputW = null;
            switch (type) {
                  case kDisplacement:
                  default:
                        pidInputW = new FuntionalPIDSource.Displacement(func);
                        break;
                  case kRate:
                        pidInputW = new FuntionalPIDSource.Rate(func);
                        break;
            }
            return pidInputW;
      }

      private static abstract class FuntionalPIDSource implements PIDSource {
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
}
