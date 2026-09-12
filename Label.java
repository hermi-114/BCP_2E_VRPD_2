
import java.math.BigInteger;

public class Label {
    public int node;
    public RCSPArc arc;
    public double duration;
    public double capacity;
    public double reducedCost;
    public Label predecessor;
    public BigInteger customerServed;
    public int d;

    public Label(int node) {
        this.node = node;
        this.arc = null;
        this.duration = 0;
        this.capacity = 0;
        this.reducedCost = 0;
        this.predecessor = null;
        this.customerServed = BigInteger.ZERO;
        this.d = 0;
    }
}
