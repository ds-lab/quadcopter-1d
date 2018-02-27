package pidexample;

import org.apache.commons.math3.exception.DimensionMismatchException;
import org.apache.commons.math3.exception.MaxCountExceededException;
import org.apache.commons.math3.ode.FirstOrderDifferentialEquations;
import org.apache.commons.math3.ode.FirstOrderIntegrator;
import org.apache.commons.math3.ode.nonstiff.ClassicalRungeKuttaIntegrator;
import org.apache.commons.math3.ode.sampling.StepHandler;
import org.apache.commons.math3.ode.sampling.StepInterpolator;
import org.apache.commons.math3.util.FastMath;
import org.jfree.chart.*;
import org.jfree.chart.renderer.xy.XYLineAndShapeRenderer;
import org.jfree.data.xy.*;
import org.jfree.ui.RefineryUtilities;

import javax.swing.*;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;
import java.awt.*;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Analyze data in R with
 * <p>
 * d <- read.csv2('c:/Users/Adriano/Desktop/data.csv', col.names = c('t', 'z', 'v'))
 * ggplot(d, aes(x = t, y = z)) + geom_line() + geom_hline(yintercept = 0, col='red', size=2)
 *
 * @author Adrian Rumpold (a.rumpold@ds-lab.org)
 */
public class Quadcopter1D implements FirstOrderDifferentialEquations, ChangeListener {
    // Indicates if the simulation is currently running
    private final AtomicBoolean inSimulation = new AtomicBoolean(false);

    // GUI components
    private final LineChart chart;
    private final ControlGui controls;
    private final JLabel pValue = new JLabel();
    private final JLabel iValue = new JLabel();
    private final JLabel dValue = new JLabel();
    private final JLabel timeValue = new JLabel();

    /* Physical properties of the quadcopter */
    private static final double GRAVITY_CONST = 9.81;   // m/s^2
    private final double mass = 0.18;   // kg
    private final double rotorLength = 0.086;   // m
    private final double minThrust = 0;
    private final double maxThrust = 1.2 * mass * GRAVITY_CONST;

    private final double z0 = 2; // Initial altitude
    private double thrust = 0; // Initial thrust

    private PIDController controller;
    private double lastTime = 0;

    // User-changeable settings
    private double kp;
    private double ki;
    private double kd;
    private double simulationTime = 10;

    private Quadcopter1D() {
        chart = new LineChart("1D Quadcopter", null);
        controls = new ControlGui();
    }

    @Override
    public void stateChanged(ChangeEvent e) {
        final JSlider source = (JSlider) e.getSource();
        final int value = source.getValue();

        // Update parameters from user input and re-run simulation
        switch (source.getName()) {
            case "time": {
                simulationTime = value;
                break;
            }
            case "kp": {
                kp = value;
                break;
            }
            case "ki": {
                ki = value / 10.0;
                break;
            }
            case "kd": {
                kd = value / 10.0;
                break;
            }
        }

        timeValue.setText(String.format("%.1f", simulationTime));
        pValue.setText(String.format("%.1f", kp));
        iValue.setText(String.format("%.1f", ki));
        dValue.setText(String.format("%.1f", kd));

        performSimulation();
    }

    private void start() {
        chart.pack();
        controls.pack();

        RefineryUtilities.centerFrameOnScreen(chart);
        RefineryUtilities.positionFrameOnScreen(controls, .8, .5);

        chart.setVisible(true);
        controls.setVisible(true);

        chart.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
        controls.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
    }


    /**
     * Calls the controller to generate the output based on the input.
     *
     * @param deltaT the time that has passed
     * @param z      the z-coordinate of the 1D-copter
     * @param vZ     the velocity of the 1D-copter
     */
    private void control(double deltaT, double z, double vZ) {
        if (deltaT == 0) {
            return;
        }

        double thrustUnclamped = controller.update(deltaT, z);

        // Clamp to physical limits
        thrust = FastMath.min(FastMath.max(thrustUnclamped, minThrust), maxThrust);
        if (thrustUnclamped != thrust) {
            System.out.println("Clamped thrust from " + thrustUnclamped + " to " + thrust);
        }
    }

    @Override
    public int getDimension() {
        return 2;
    }

    /**
     * Calculate the derivatives of the state vector for an ODE integrator time step.
     * <p>
     * {@code y} is the state vector, containing the following entries:
     * <ul>
     * <li>{@code y[0]}: Position along the z axis (= altitude)</li>
     * <li>{@code y[1]}: Velocity along the z axis</li>
     * </ul>
     * <p>
     * {@code yDot} is calculated by this method and contains the derivatives of
     * the state vector:
     * <ul>
     * <li>{@code yDot[0]}: contains the derivative of the z position y[0] (--> velocity in z direction)</li>
     * <li>{@code yDot[1]}: contains the derivative of the z velocity y[1] (--> total acceleration in z direction)</li>
     * </ul>
     *
     * @param t    current integration time
     * @param y    state vector as calculated in the last integration time step
     * @param yDot vector of derivatives for the current time step, calculated by this method
     * @throws MaxCountExceededException
     * @throws DimensionMismatchException
     */
    @Override
    public void computeDerivatives(double t, double[] y, double[] yDot) throws MaxCountExceededException, DimensionMismatchException {
        // Invoke control loop
        control(t - lastTime, y[0], y[1]);
        lastTime = t;

        /*
         */
        yDot[0] = y[1];
        yDot[1] = thrust / mass - GRAVITY_CONST;
    }

    public void performSimulation() {
        if (inSimulation.compareAndSet(false, true)) {
            return;
        }

        System.out.printf("Performing simulation with: t = %.2f, Kp = %.2f, Ki = %.3f, Kd = %.3f\n",
                simulationTime, kp, ki, kd);

        controller = new PIDController(new PIDParameters(kp, ki, kd, 1.0, minThrust, maxThrust));

        // Set up the plot and remove any existing data
        chart.getChart().getXYPlot().getDomainAxis().setRange(0, simulationTime);
        final XYSeries series = chart.getDataset().getSeries(0);
        final XYSeries setpointSeries = chart.getDataset().getSeries(1);
        series.clear();
        setpointSeries.clear();

        // Create an ODE integrator for the IVP
        final FirstOrderIntegrator integrator = new ClassicalRungeKuttaIntegrator(1e-2);
        double[] y = new double[]{z0, 0};

        lastTime = 0;
        thrust = 0;
        controller.setSetpoint(z0);

        // Add code to be executed for every time step of the ODE integrator
        integrator.addStepHandler(new StepHandler() {
            @Override
            public void init(double t0, double[] y0, double t) {
                output(t0, y0);
            }

            @Override
            public void handleStep(StepInterpolator interpolator, boolean isLast) throws MaxCountExceededException {
                double t = interpolator.getCurrentTime();
                double[] y = interpolator.getInterpolatedState();

                // Change the setpoint during the simulation to see step response
                if (t > 5) {
                    controller.setSetpoint(4, false);
                }

                output(t, y);
            }

            private void output(double t, double[] y) {
                // Add the current data point to the plot
                try {
                    series.add(t, y[0]);
                    setpointSeries.add(t, (double) controller.getSetpoint());
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        });

        // Solve the IVP on the defined time interval
        integrator.integrate(this, 0, y, simulationTime, y);
        inSimulation.set(false);
    }

    public static void main(String[] args) {
        Locale.setDefault(Locale.GERMAN);
        final Quadcopter1D sim = new Quadcopter1D();
        sim.start();
    }

    private class LineChart extends JFrame {
        private static final long serialVersionUID = 1L;
        private final XYSeriesCollection dataset = new XYSeriesCollection();
        private final JFreeChart chart;

        public LineChart(String appTitle, String chartTitle) {
            super(appTitle);
            dataset.addSeries(new XYSeries("Altitude"));
            dataset.addSeries(new XYSeries("Setpoint"));

            chart = createChart(dataset, chartTitle);
            chart.setAntiAlias(true);

            final ChartPanel panel = new ChartPanel(chart);
            panel.setPreferredSize(new Dimension(800, 400));
            setContentPane(panel);
        }

        private JFreeChart createChart(XYDataset dataset, String chartTitle) {
            final JFreeChart chart = ChartFactory.createScatterPlot(chartTitle, "Time", "Measurement", dataset);
            chart.getXYPlot().setRenderer(new XYLineAndShapeRenderer(true, false));
            return chart;
        }

        public XYSeriesCollection getDataset() {
            return dataset;
        }

        public JFreeChart getChart() {
            if (chart != null) {
                return chart;
            } else {
                return null;
            }
        }
    }

    private class ControlGui extends JFrame {
        private static final long serialVersionUID = 1L;

        public ControlGui() throws HeadlessException {
            super("PID control parameters");
            final GridBagLayout layout = new GridBagLayout();
            setLayout(layout);

            final GridBagConstraints sliderConstraint = new GridBagConstraints();
            sliderConstraint.gridx = GridBagConstraints.RELATIVE;
            sliderConstraint.gridy = 0;
            sliderConstraint.weightx = 1;
            sliderConstraint.fill = GridBagConstraints.HORIZONTAL;

            final GridBagConstraints labelConstraints = new GridBagConstraints();
            labelConstraints.gridx = GridBagConstraints.RELATIVE;
            labelConstraints.gridy = 0;
            labelConstraints.ipadx = 10;
            labelConstraints.ipady = 4;

            final JLabel pLabel = new JLabel("Kp = ");
            final JSlider pSlider = new JSlider(0, 50, 0);
            add(pLabel, labelConstraints);
            pSlider.setName("kp");
            pSlider.addChangeListener(Quadcopter1D.this);
            add(pSlider, sliderConstraint);
            add(pValue, labelConstraints);

            ++sliderConstraint.gridy;
            ++labelConstraints.gridy;

            final JLabel iLabel = new JLabel("Ki = ");
            final JSlider iSlider = new JSlider(0, 100, 0);
            add(iLabel, labelConstraints);
            iSlider.setName("ki");
            iSlider.addChangeListener(Quadcopter1D.this);
            add(iSlider, sliderConstraint);
            add(iValue, labelConstraints);

            ++sliderConstraint.gridy;
            ++labelConstraints.gridy;

            final JLabel dLabel = new JLabel("Kd = ");
            final JSlider dSlider = new JSlider(0, 100, 0);
            add(dLabel, labelConstraints);
            dSlider.setName("kd");
            dSlider.addChangeListener(Quadcopter1D.this);
            add(dSlider, sliderConstraint);
            add(dValue, labelConstraints);

            ++sliderConstraint.gridy;
            ++labelConstraints.gridy;

            final JLabel timeLabel = new JLabel("Simulation length");
            final JSlider lengthSlider = new JSlider(0, 60, (int) simulationTime);
            add(timeLabel, labelConstraints);
            lengthSlider.setName("time");
            lengthSlider.addChangeListener(Quadcopter1D.this);
            add(lengthSlider, sliderConstraint);
            add(timeValue, labelConstraints);

            stateChanged(new ChangeEvent(lengthSlider));
        }
    }
}
