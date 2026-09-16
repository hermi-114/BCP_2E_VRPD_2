import java.math.BigInteger;

public class Label {
    public int node;
    public RCSPArc arc;
    public double duration;
    public double capacity;
    public double reducedCost;
    public Label predecessor;
    public BigInteger ngSet;
    public int d;
    public int droneUsed;   // <-- ADD THIS

    public double[] r1cState;

    public Label(int node) {
        this.node = node;
        this.arc = null;
        this.duration = 0;
        this.capacity = 0;
        this.reducedCost = 0;
        this.predecessor = null;
        this.ngSet = BigInteger.ZERO;
        this.d = 0;
        this.droneUsed = 0;
    }
}