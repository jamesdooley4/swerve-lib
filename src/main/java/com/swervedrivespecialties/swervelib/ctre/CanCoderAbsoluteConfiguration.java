package com.swervedrivespecialties.swervelib.ctre;

public class CanCoderAbsoluteConfiguration {
    private final int id;
    private final double offset;
    private final String canBus;

    public CanCoderAbsoluteConfiguration(int id, double offset, String canBus) {
        this.id = id;
        this.offset = offset;
        this.canBus = canBus;
    }

    public CanCoderAbsoluteConfiguration(int id, double offset) {
        this.id = id;
        this.offset = offset;
        this.canBus = "rio";
    }

    public int getId() {
        return id;
    }

    public double getOffset() {
        return offset;
    }

    public String getCanBus() {
        return canBus;
    }
}
