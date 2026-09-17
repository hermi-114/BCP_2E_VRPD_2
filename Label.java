import java.math.BigInteger;

public class Label {
    int node;
    double duration;
    double capacity;
    double reducedCost;
    int droneUsed;
    int d;

    BigInteger ngSet;
    BigInteger customerServed;      // union of truckBit and droneBit

    // ---- NEW: branch state ----
    BigInteger truckBit = BigInteger.ZERO;   // customers visited by truck in this label
    BigInteger droneBit = BigInteger.ZERO;   // customers served by drone in this label

    Label predecessor;
    RCSPArc arc;
    double[] r1cState;

    Label(int node) {
        this.node = node;
        this.ngSet = BigInteger.ZERO;
        this.customerServed = BigInteger.ZERO;
        this.truckBit = BigInteger.ZERO;
        this.droneBit = BigInteger.ZERO;
    }
}