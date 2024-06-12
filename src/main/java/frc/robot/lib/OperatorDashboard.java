package frc.robot.lib;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class OperatorDashboard {
    ShuffleboardTab dashboard;
    private final SimpleWidget intakeFrontRPMoffset;
    private final SimpleWidget intakeRearRPMoffset;


    private final SimpleWidget shooterTopRPMoffset;
    private final SimpleWidget shooterBottomRPMoffset;


    public OperatorDashboard(){
        dashboard = Shuffleboard.getTab("Dashboard");

        intakeFrontRPMoffset = dashboard
                .add("Front Intake RPM", 0.0)
                .withSize(12, 3)
                .withPosition(0, 4)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 8000.0, "block increment", 50));

        intakeRearRPMoffset = dashboard
                .add("Rear Intake RPM", 0.0)
                .withSize(12, 3)
                .withPosition(0, 4)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 8000.0, "block increment", 50));


        shooterTopRPMoffset = dashboard
                .add("Shooter Top RPM", 0.0)
                .withSize(12, 3)
                .withPosition(0, 4)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 8000.0, "block increment", 50));

        shooterBottomRPMoffset = dashboard
                .add("Shooter Bottom RPM", 0.0)
                .withSize(12, 3)
                .withPosition(0, 4)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 8000.0, "block increment", 50));

    }

    public double getFrontIntakeRPM() {
        return intakeFrontRPMoffset.getEntry().getDouble(0.0);
    }

    public double getRearIntakeRPM() {
        return intakeRearRPMoffset.getEntry().getDouble(0.0);
    }

    public double getBottomShooterRPM() {
        return shooterBottomRPMoffset.getEntry().getDouble(0.0);
    }

    public double getTopShooterRPM() {
        return shooterTopRPMoffset.getEntry().getDouble(0.0);
    }
}
