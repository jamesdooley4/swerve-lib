package com.swervedrivespecialties.swervelib.ctre;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.swervedrivespecialties.swervelib.DriveController;
import com.swervedrivespecialties.swervelib.DriveControllerFactory;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import edu.wpi.first.wpilibj.RobotBase;

public final class Falcon500DriveControllerFactoryBuilder {
    private static final double TICKS_PER_ROTATION = 2048.0;

    private static final int CAN_TIMEOUT_MS = 250;
    private static final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;

    private double nominalVoltage = Double.NaN;
    private double currentLimit = Double.NaN;

    public Falcon500DriveControllerFactoryBuilder withVoltageCompensation(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public DriveControllerFactory<ControllerImplementation, Integer> build() {
        return new FactoryImplementation();
    }

    public Falcon500DriveControllerFactoryBuilder withCurrentLimit(double currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }

    private class FactoryImplementation implements DriveControllerFactory<ControllerImplementation, Integer> {
        @Override
        public ControllerImplementation create(Integer driveConfiguration, ModuleConfiguration moduleConfiguration, String canBus) {
            TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

            double sensorPositionCoefficient = Math.PI * moduleConfiguration.getWheelDiameter() * moduleConfiguration.getDriveReduction() / TICKS_PER_ROTATION;
            double sensorVelocityCoefficient = sensorPositionCoefficient * 10.0;

            if (hasVoltageCompensation()) {
                motorConfiguration.voltageCompSaturation = nominalVoltage;
            }

            if (hasCurrentLimit()) {
                motorConfiguration.supplyCurrLimit.currentLimit = currentLimit;
                motorConfiguration.supplyCurrLimit.enable = true;
            }

            WPI_TalonFX motor = new WPI_TalonFX(driveConfiguration, canBus);
            motor.configAllSettings(motorConfiguration);
            CtreUtils.checkCtreError(motor.configAllSettings(motorConfiguration), "Failed to configure Falcon 500");

            if (hasVoltageCompensation()) {
                // Enable voltage compensation
                motor.enableVoltageCompensation(true);
            }

            motor.setNeutralMode(NeutralMode.Brake);

            motor.setInverted(moduleConfiguration.isDriveInverted() ? TalonFXInvertType.Clockwise : TalonFXInvertType.CounterClockwise);
            motor.setSensorPhase(true);

            // Reduce CAN status frame rates
            CtreUtils.checkCtreError(
                    motor.setStatusFramePeriod(
                            StatusFrameEnhanced.Status_1_General,
                            STATUS_FRAME_GENERAL_PERIOD_MS,
                            CAN_TIMEOUT_MS
                    ),
                    "Failed to configure Falcon status frame period"
            );

            return new ControllerImplementation(motor, sensorVelocityCoefficient);
        }
    }

    private class ControllerImplementation implements DriveController {
        private final WPI_TalonFX motor;
        private final double sensorVelocityCoefficient;
        private final double nominalVoltage = hasVoltageCompensation() ? Falcon500DriveControllerFactoryBuilder.this.nominalVoltage : 12.0;

        private ControllerImplementation(WPI_TalonFX motor, double sensorVelocityCoefficient) {
            this.motor = motor;
            this.sensorVelocityCoefficient = sensorVelocityCoefficient;
        }

        @Override
        public Object getDriveMotor() {
            return this.motor;
        }

        @Override
        public void setReferenceVoltage(double voltage) {
            double percentOutput = voltage / nominalVoltage;
            motor.set(TalonFXControlMode.PercentOutput, percentOutput);
            if (RobotBase.isSimulation()) {
                if (motor.getInverted()) {
                    percentOutput *= -1.0;
                }

                // SelectedSensorVelocity is raw sensor units per 100ms
                // See https://store.ctr-electronics.com/content/api/java/html/interfacecom_1_1ctre_1_1phoenix_1_1motorcontrol_1_1_i_motor_controller.html#a2e40db44cfbd62192ffac3fb7ccf5166
                // Raw sensor units are 2048 ticks per rotation
                // See https://docs.ctre-phoenix.com/en/stable/ch14_MCSensor.html#sensor-resolution
                // Max RPM is ~6000 per https://motors.vex.com/vexpro-motors/falcon
                // Synthetic velocity is outputPercent * 2048 (ticks/rotation) * 6000 RPM / 60 (sec/min) / 10 (100ms/sec)
                motor.getSimCollection().setIntegratedSensorVelocity((int)(percentOutput * 2048.0 * 6000.0 / 60.0 / 10.0));
            }
        }

        @Override
        public double getStateVelocity() {
            return motor.getSelectedSensorVelocity() * sensorVelocityCoefficient;
        }
    }
}
