package pidexample;

/**
 * A PID controller with optional output bounds, output moving average filter
 * and integral error clamping.
 * <p>
 * Instances should be created through the Builder pattern using the
 * {@link #withGains(double, double, double)} method or by passing a set of
 * parameters to the {@link #PIDController(PIDParameters)} constructor.
 *
 * @author Adrian Rumpold (a.rumpold@gmail.com)
 * @see PIDParameters
 */
public class PIDController {
    private double kp; // Proportional gain
    private double ki; // Integral gain
    private double kd; // Differential gain

    private double alpha = 1.0f; // Low-pass filter ratio

    private Double minOutput = null;
    private Double maxOutput = null;
    private Double setpoint = null;
    private double maxIntegral = Double.MAX_VALUE;
    private double integral = 0.0f;
    private double lastError;
    private double lastOutput = Double.NaN;

    private boolean debug = false;

    /**
     * Default constructor inaccessible - use Builder methods
     */
    private PIDController() {
    }

    /**
     * Construct a new PID controller with the supplied set of parameters.
     *
     * @param params
     */
    public PIDController(PIDParameters params) {
        this.kp = params.getKp();
        this.kd = params.getKd();
        this.ki = params.getKi();

        this.minOutput = params.getMinOutput();
        this.maxOutput = params.getMaxOutput();
        this.alpha = params.getAlpha();
    }

    /**
     * Change the output value by applying the moving average filter
     */
    private double filter(double output) {
        throw new RuntimeException("update() -- not implemented");
    }

    /**
     * Feed the current input to the controller and obtain updated output.
     *
     * @param current the current input value
     * @return the controller output after applying optional bounds clamping and
     * output filtering
     * @throws IllegalStateException if the controller does not have a setpoint
     * @oaram deltaT elapsed time in seconds since the last update
     */
    public double update(double deltaT, double current) {
        throw new RuntimeException("update() -- not implemented");
    }

    /**
     * @return the output moving average filter coefficient
     */
    public double getFilterRatio() {
        return alpha;
    }

    /**
     * @return the optional lower output bound
     */
    public Double getMinOutput() {
        return minOutput;
    }

    /**
     * @return the optional upper output bound
     */
    public Double getMaxOutput() {
        return maxOutput;
    }

    /**
     * Check if this controller has a setpoint
     *
     * @return the setpoint state
     */
    public boolean hasSetpoint() {
        return setpoint != null;
    }

    /**
     * @return the setpoint value
     */
    public Double getSetpoint() {
        return setpoint;
    }

    /**
     * Update the controller setpoint value
     *
     * @param setpoint the new setpoint
     * @param reset    indicates if the integral error should be reset to 0
     */
    public void setSetpoint(double setpoint, boolean reset) {
        if (reset) {
            integral = 0.0;
        }
        this.setpoint = setpoint;
    }

    /**
     * Update the controller setpoint value
     * <p/>
     * Note, that this operation resets the integral error to 0
     *
     * @param setpoint the new setpoint
     */
    public void setSetpoint(double setpoint) {
        setSetpoint(setpoint, true);
    }

    /**
     * @return the current integral error
     */
    public double getIntegral() {
        return integral;
    }
}
