
import java.math.BigInteger;

public class RCSPArc {
    public int src, dst;
    public double duration;
    public double capacity;
    public double reducedCost;
    public BigInteger customerServed;
    public DroneSchedule schedule;

    public RCSPArc(int src, int dst, double duration, double capacity, double reducedCost, BigInteger customerServed, DroneSchedule schedule) {
        this.capacity = capacity;
        this.customerServed = customerServed;
        this.dst = dst;
        this.duration = duration;
        this.schedule = schedule;
        this.src = src;
    }

}
