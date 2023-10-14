package frc.robot;

public class Positioner {
    private enum Positions {
        Flat,
        Shoot
    }

    private Arm m_arm;

    Positioner(Arm arm) {
        m_arm = arm;
    };

    public void position() {

    }
}
