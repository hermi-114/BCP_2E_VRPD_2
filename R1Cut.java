
import java.util.Set;

public class R1Cut implements ICut {
    public Set<Integer> subsetC;

    @Override
    public double getCoefficientForRoute(Route route) {
        throw new UnsupportedOperationException("Not supported yet.");
    }

    @Override
    public double getReducedCostPenaltyForTruckArc(int src, int dst, int numDrones) {
        throw new UnsupportedOperationException("Not supported yet.");
    }

    @Override
    public double getReducedCostPenaltyForDroneArc(int park, DroneSchedule schedule, int numDrones) {
        throw new UnsupportedOperationException("Not supported yet.");
    }

    @Override
    public double getDual() {
        throw new UnsupportedOperationException("Not supported yet.");
    }

    @Override
    public void setDual(double newDual) {
        throw new UnsupportedOperationException("Not supported yet.");
    }

    @Override
    public double getRHS() {
        throw new UnsupportedOperationException("Not supported yet.");
    }

    
    
}
