package frc.lib.util;

// my super simple linear interpolation class
public class LinearInterpolate {
    private final double[] x;
    private final double[] y;

    /**
     * simple linear interpolation between points
     * with increasing x values.
     * Not fancy, but for aiming this is all we need :)
     * It's on you to make sure the x values are in order
     * and there are the same number of x and y values, good luck
     * @param x array with x function values
     * @param y array with corresponding y function values
     */
    public LinearInterpolate (double[] x, double[] y) {
        this.x = x;
        this.y = y;
    }

    public double interpolate (double x) {
        int i = findX(x);
        return interpolate(x, this.x[i], this.x[i + 1], this.y[i], this.y[i + 1]);
    }

    public double derivative (double x) {
        int i = findX(x);
        return (this.y[i + 1] - this.y[i]) / (this.x[i + 1] - this.x[i]);
    }

    public static double interpolate (double x, double x0, double x1, double y0, double y1) {
        return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
    }

    private int findX (double x) {
        for(int i = 0; i < this.x.length - 1; i++) {
            if(this.x[i] <= x && x <= this.x[i + 1]) return i;
        }

        return (x < this.x[0]) ? 0 : this.x.length - 2;
    }

    public static void main(String[] args) {
        double[] x = {0, 1, 2, 3, 4, 5};
        double[] y = {0, 6, 2, 5, 4, 4};

        LinearInterpolate li = new LinearInterpolate(x, y);

        for(double i = -1; i < 6; i += 0.1) {
            System.out.println(i + "," + li.interpolate(i));
        }
    }
}
