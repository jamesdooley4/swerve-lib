package com.swervedrivespecialties.swervelib;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

@FunctionalInterface
public interface DriveControllerFactory<Controller extends DriveController, DriveConfiguration> {
    default void addDashboardEntries(
            ShuffleboardContainer container,
            Controller controller
    ) {
        container.addNumber("Current Velocity", controller::getStateVelocity);
    }

    default Controller create(
            ShuffleboardContainer container,
            DriveConfiguration driveConfiguration,
            ModuleConfiguration moduleConfiguration,
            String canBus
    ) {
        var controller = create(driveConfiguration, moduleConfiguration, canBus);
        addDashboardEntries(container, controller);

        return controller;
    }

    Controller create(DriveConfiguration driveConfiguration, ModuleConfiguration moduleConfiguration, String canBus);
}
