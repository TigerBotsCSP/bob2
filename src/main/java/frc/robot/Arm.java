package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class Arm {
    private CANSparkMax m_leftIntaker;
    private CANSparkMax m_rightIntaker;

    // Public for testing encoders
    public CANSparkMax m_rotation;
    public CANSparkMax m_rightRotation;

    // Positioning PID
    public PIDController m_positionPID = new PIDController(0.1, 0.01, 0);

    public boolean m_in = false;
    public boolean m_shootingOut = false;
    public boolean m_positioning = false;

    Arm() {
        m_leftIntaker = new CANSparkMax(11, MotorType.kBrushless);
        m_rightIntaker = new CANSparkMax(10, MotorType.kBrushless);
        //Right spins inverse of left
        m_rightIntaker.follow(m_leftIntaker, true);

        m_rotation = new CANSparkMax(13, MotorType.kBrushless);
    }

    // Toggle the intaker
    // If it's going in, make it go out, and vise-versa
    public void toggleIntaker() {
        if (m_shootingOut) return;

        if (m_in) {
            m_leftIntaker.set(ArmConstants.kIntakerSpeed);
            m_in = false;
        } else {
            m_leftIntaker.set(-ArmConstants.kIntakerSpeed);
            m_in = true;
        }
    }

    // Shoots out a cube
    public void shootOut() {
        if (m_shootingOut) return;

        try {
            ExecutorService executor = Executors.newSingleThreadExecutor();
            executor.submit(() -> {
                m_shootingOut = true;
    
                m_leftIntaker.set(-1);
                Timer.delay(.2);
                m_leftIntaker.set(m_in ? -ArmConstants.kIntakerSpeed : ArmConstants.kIntakerSpeed);
    
                m_shootingOut = false;
            });
        } catch(Exception e) {
            System.out.println(e.getMessage());
        }
    }

    // Set the rotation of the arm using the joystick value
    public void setRotation(double speed) {
        if (m_positioning) return;

        m_rotation.set(-Math.max(Math.min(speed, ArmConstants.kRotationSpeed), -ArmConstants.kRotationSpeed));
    }

    Timer m_timer = new Timer();

    public void position() {
        if (m_positioning) return;

        ExecutorService executor = Executors.newSingleThreadExecutor();
        executor.submit(() -> {
            m_positioning = true;

            m_timer.start();

            while (!m_positionPID.atSetpoint() && m_positioning) {
                double value = m_positionPID.calculate(m_rotation.getEncoder().getPosition(), ArmConstants.kShootingAngle);

                m_rotation.set(value * .3);

                SmartDashboard.putNumber("Positioner", value);
                SmartDashboard.putNumber("Time", m_timer.get());

                if (Math.abs(value) < .02) break;
                if (m_timer.get() 
                > .5) break;
            }

            m_positionPID.reset();
            m_timer.stop();
            m_timer.reset();
            m_positioning = false;
        });
    }
}