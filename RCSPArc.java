import java.math.BigInteger;
import java.util.ArrayList;
import java.util.List;

public class RCSPArc {
    public int src, dst;
    public double duration;
    public double capacity;
    public double reducedCost;
    public BigInteger ngSet;                     // served customers as a bitmask
    public DroneSchedule schedule;

    // BUG FIX: added to preserve the order of visited customers, needed for
    // (a) ng-set updates (^ = (^ ∩ N_v) ∪ {v}) and (b) route reconstruction.
    public List<Integer> servedCustomersInOrder;

    public RCSPArc(int src, int dst,
                   double duration, double capacity,
                   double reducedCost,
                   BigInteger ngSet,
                   List<Integer> servedCustomersInOrder,
                   DroneSchedule schedule) {
        this.src                      = src;
        this.dst                      = dst;
        this.duration                 = duration;
        this.capacity                 = capacity;
        this.reducedCost              = reducedCost;   // BUG FIX: was missing
        this.ngSet                    = ngSet;
        this.servedCustomersInOrder   = (servedCustomersInOrder != null)
                                        ? servedCustomersInOrder
                                        : new ArrayList<>();
        this.schedule                 = schedule;
    }
}