
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class R1Cut implements ICut {
    public final Set<Integer> subsetC;
    public final int rhs;
    private final Map<Integer, Double> rhoMap;
    public double dual = 0;

    public R1Cut(Set<Integer> C) {
        
        this.subsetC = new HashSet<>(C);
        this.rhoMap = new HashMap<>();
        for(int cust : C) rhoMap.put(cust, 0.5);

        double sumRho = C.size() * 0.5;
        this.rhs = (int) Math.floor(sumRho + Constant.EPSILON);
    }
    
    @Override
    public double getDual() {
        return dual;
    }

    @Override
    public void setDual(double newDual) {
        this.dual = newDual;
    }

    @Override
    public double getRHS() {
        return this.rhs;
    }

    @Override
    public Set<Integer> getSubsetC() {
        return this.subsetC;
    }

    @Override
    public double getCoefficientForRoute(Route route) {
        List<Integer> path = new ArrayList<>();
        for(Node cust : route.sequence) path.add(cust.id);
        path.set(path.size()-1, -1); // marker for 0'

        double s = 0;
        for(int node : path) {
            if(rhoMap.containsKey(node)) s += rhoMap.get(node);
        }

        return Math.floor(s + Constant.EPSILON);
    }

    @Override
    public double getReducedCostPenaltyForTruckArc(int src, int dst, int numDrones) {
        
        if(rhoMap.containsKey(dst)) return -dual * rhoMap.get(dst);
        return 0;
    }

    @Override
    public double getReducedCostPenaltyForDroneArc(int park, DroneSchedule schedule, int numDrones) {
        double penalty = 0;

        for(int cust : schedule.customerServed) {
            if(rhoMap.containsKey(cust)) penalty += -dual * rhoMap.get(cust);
        }

        return penalty;
    }

}
