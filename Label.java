import java.math.BigInteger;

public class Label {
    int node;
    double duration;
    double capacity;
    double reducedCost;
    int droneUsed;
    int d;

    BigInteger ngSet;
    BigInteger customerServed;

    // ---- arc-forcing resource (only used when branching forces an arc) ----
    public boolean requiredArcUsed = false;

    Label predecessor;
    RCSPArc arc;
    double[] r1cState;

    Label(int node) {
        this.node = node;
        this.ngSet = BigInteger.ZERO;
        this.customerServed = BigInteger.ZERO;
    }
}