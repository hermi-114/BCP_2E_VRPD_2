
import java.util.Set;

public class ARCCut implements ICut {
    public final Set<Integer> subsetC;
    public final int rhs;
    public final int k;
    public double dual = 0;
    
    public ARCCut(Set<Integer> C) {
        this.subsetC = C;
        
        double totalDemand = subsetC.stream()
        .mapToInt(cust -> VRPInstance.nodes.get(cust).demand)
        .sum();
        
        this.k = (int) Math.floor(Constant.TRUCK_PAYLOAD / Constant.DRONE_AND_EQUIPMENT_WEIGHT);
        this.rhs = (int) Math.ceil(totalDemand / Constant.DRONE_AND_EQUIPMENT_WEIGHT - Constant.EPSILON);
    }
    
    @Override
    public void setDual(double newDual) { this.dual = newDual; }
    
    @Override
    public double getRHS() { return this.rhs; }
    
    @Override
    public Set<Integer> getSubsetC() { return this.subsetC; }

    @Override
    public double getCoefficientForRoute(Route route) {
        int h_r = 0;

        for(int i = 1; i < route.sequence.size(); i++) {
            int pre = route.sequence.get(i-1).id;
            int cur = route.sequence.get(i).id;

            if(!subsetC.contains(pre) && subsetC.contains(cur)) h_r++;
        }

        for(var entry : route.customerDroneSchedule.entrySet()) {
            for(int cust : entry.getValue().customerServed) {
                if(subsetC.contains(cust)) h_r++;
            }
        }

        return h_r * (k - route.getNumDrone());
    }

    @Override
    public double getReducedCostPenaltyForTruckArc(int src, int dst, int numDrones) {
        if (!subsetC.contains(src) && subsetC.contains(dst))
            return -dual * (k - numDrones);
        return 0.0;
    }

    @Override
    public double getReducedCostPenaltyForDroneArc(int park, DroneSchedule schedule, int numDrones) {
        for (int cust : schedule.customerServed) {
            if (subsetC.contains(cust))
                return -dual * (k - numDrones);
        }
        return 0.0;
    }

    @Override
    public double getDual() { return this.dual; }



}
