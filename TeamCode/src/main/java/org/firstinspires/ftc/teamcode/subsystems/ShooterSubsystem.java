package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * ShooterSubsystem: computes muzzle speed, maps to flywheel RPM, controls flywheel with a PI controller, and controls a servo feeder in async
 *
 * IMPORTANT: tune RPM_PER_MS, RPM_OFFSET, kP, kI, and servo positions from hardware later
 */
public class ShooterSubsystem {
    //hardware
    private final DcMotorEx flywheel;
    private final Servo feeder;

    //timing
    private final ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0.0;

    //physics constants
    private final double g = 9.80665; //accurate constant for gravity (m/s^2)
    private double launchAngleRad = Math.toRadians(60.0); //launch angle of the shooter (degrees, converted to radians)
    private double shooterHeightMeters = 0.40; //height of shooter from the floor (meters)

    //FIXME: placeholder values for a typical 4" wheel shooter
    //needs to be tuned with lin regression (ball spd->rpm)
    private double RPM_PER_MS = 120.0; //rpm per (m/s)
    private double RPM_OFFSET = 0.0; //rpm offset (should be small)

    //motor/encoder
    private final double TICKS_PER_REV;

    //controller (PI)
    private double kP = 0.0008; //prop gain
    private double kI = 0.0006; //int gain
    private double integrator = 0.0; //integral term
    private double integratorMin = -1.0, integratorMax = 1.0; //integrator lims
    private double maxPower = 1.0; //max motor pow

    //tolerance for "at target"
    private double rpmTolerance = 50.0; //rpm

    //state
    private double targetRPM = 0.0; //desired rpm

    //feeder state machine (async)
    private enum FeederState {IDLE, PULSE_OUT, WAIT_RETRACT}
    private FeederState feederState = FeederState.IDLE;
    private double feederStateStart = 0.0; //time when we entered state
    private long feederPulseMs = 150; //pulse duration for "out"
    private long feederRetractDelayMs = 120; //how long to wait before retract
    private double feederStartPos = 0.0; //servo start (retracted) position
    private double feederEndPos = 1.0; //servo pushed position

    public ShooterSubsystem(HardwareMap hw, String flywheelName, String feederServoName) {
        flywheel = hw.get(DcMotorEx.class, flywheelName);
        //allow using dcmotor if user only has dcmotor configured; convert if needed
        if (flywheel==null) throw new IllegalArgumentException("Flywheel motor (DcMotorEx) not found: "+flywheelName);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        TICKS_PER_REV = flywheel.getMotorType().getTicksPerRev();
        if (feederServoName!=null&&!feederServoName.isEmpty()) feeder = hw.get(Servo.class, feederServoName);
        else feeder = null; //no feeder
        lastTime = timer.seconds(); //update timer with last time
    }

    //config setters for tuning
    public void setLaunchAngleDegrees(double deg) {launchAngleRad = Math.toRadians(deg);}
    public void setShooterHeightMeters(double h) {shooterHeightMeters = h;}
    public void setRegressionCoeffs(double slopeRpmPerMs, double offsetRpm) {
        this.RPM_PER_MS = slopeRpmPerMs;
        this.RPM_OFFSET = offsetRpm;
    }
    public void setGains(double p, double i) { this.kP = p; this.kI = i; }
    public void setRpmTolerance(double tol) { this.rpmTolerance = tol; }
    public void setFeederTimings(long pulseMs, long retractDelayMs) {
        this.feederPulseMs = pulseMs;
        this.feederRetractDelayMs = retractDelayMs;
    }
    public void setFeederPositions(double startPos, double endPos) {
        this.feederStartPos = startPos;
        this.feederEndPos = endPos;
        if (feeder!=null) feeder.setPosition(feederStartPos);
    }

    //encoder/rpm conversions
    private double encoderTicksPerSecToRPM(double ticksPerSec) {
        return ticksPerSec * 60.0/TICKS_PER_REV;
    }
    public double getFlywheelRPM() {
        double velTicksPerSec = flywheel.getVelocity(); //ticks per sec
        return encoderTicksPerSecToRPM(velTicksPerSec);
    }

    //projectile math, src:(https://medium.com/@vikramaditya.nishant/programming-a-decode-shooter-4ab114dac01f)
    public double computeRequiredBallVelocity(double distanceMeters, double goalHeightMeters) {
        double denom = 2.0*Math.cos(launchAngleRad)*Math.cos(launchAngleRad)*(distanceMeters*Math.tan(launchAngleRad)-(goalHeightMeters-shooterHeightMeters));
        if (denom<=0.0) return Double.NaN;
        double numer = g*distanceMeters*distanceMeters;
        double res = numer/denom;
        if (res<=0.0) return Double.NaN;
        return Math.sqrt(res);
    }
    public double computeRequiredBallVelocityWhileMoving(double initialDistanceMeters, double robotRelativeVelAlongGoalAxisMetersPerSec, double goalHeightMeters, int iters) {
        double x = initialDistanceMeters; //start with initial dist
        double v0 = computeRequiredBallVelocity(x, goalHeightMeters); //first guess
        if (Double.isNaN(v0)) return Double.NaN; //give up and despair
        for (int i=0; i<iters; i++) {
            double t = x/(v0*Math.cos(launchAngleRad)); //time to reach goal
            double xEffective = initialDistanceMeters-robotRelativeVelAlongGoalAxisMetersPerSec*t; //effective dist
            if (xEffective<=0.0) xEffective = 0.01; //avoid div0 or neg dist
            double newV0 = computeRequiredBallVelocity(xEffective, goalHeightMeters); //recompute
            if (Double.isNaN(newV0)) return Double.NaN; //give up and despair pt2
            v0 = newV0; x = xEffective; //update for next iter
        }
        return v0;
    }
    public double ballSpeedToTargetRPM(double ballSpeedMps) {
        return RPM_PER_MS*ballSpeedMps+RPM_OFFSET;
    }
    public void setTargetRPM(double rpm) {this.targetRPM = rpm;}
    public boolean isAtTargetRPM() {
        return Math.abs(getFlywheelRPM()-targetRPM)<=rpmTolerance;
    }

    //request a feed (async)
    public void requestFeed() {
        if (feeder==null) return; //no feeder, then despair
        if (feederState==FeederState.IDLE) { //if we idle then start
            feederState = FeederState.PULSE_OUT; //start pulse out
            feederStateStart = timer.milliseconds(); //keep track of time
            feeder.setPosition(feederEndPos); //move servo out
        }
    }
    //call to update controller and feeder state machine
    public void update() {
        double now = timer.seconds(); //current time
        double dt = now-lastTime; //time since last update
        if (dt<=0.0) dt = 1e-3; //avoid div by 0
        lastTime = now; //update last time
        //controller for flywheel
        double currentRPM = getFlywheelRPM(); //current rpm gotten
        double error = targetRPM-currentRPM; //difference in rpm
        integrator+=error*dt; //integrate error
        if (integrator>integratorMax) integrator = integratorMax; //limiting if too big
        if (integrator<integratorMin) integrator = integratorMin; //limiting if too small
        double power = kP*error+kI*integrator; //calculate power to apply to motor
        if (power>maxPower) power = maxPower; //upper bound limit to power
        if (power<-maxPower) power = -maxPower; //lower bound limit to power
        flywheel.setPower(power); //apply power to the motor
        //feeder state machine (async)
        if (feeder!=null) { //if we have a feeder
            double tMs = timer.milliseconds();
            switch (feederState) { //this switch just runs the state machine
                case IDLE:
                    //nothing
                    break;
                case PULSE_OUT:
                    if (tMs-feederStateStart>=feederPulseMs) {
                        //move to retract
                        feeder.setPosition(feederStartPos); //retract now or after short wait
                        feederState = FeederState.WAIT_RETRACT; //next state
                        feederStateStart = tMs; //update time
                    }
                    break;
                case WAIT_RETRACT:
                    if (tMs-feederStateStart>=feederRetractDelayMs) { //done waiting, retract
                        feeder.setPosition(feederStartPos); //retract to start pos
                        feederState = FeederState.IDLE; //back to idle
                    }
                    break;
            }
        }
    }
}
