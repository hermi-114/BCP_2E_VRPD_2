
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class CutGeneration {
    private static final int MAX_CUTS_PER_ROUND = 100;
    private static final double VIOLATION_THRESHOLD = 1e-3;

    public List<ICut> separateCuts(List<Route> routes, double[] lambda) {
        
        // ARCC
        List<ICut> newCuts = new ArrayList<>();
        newCuts.addAll(separateARCC(routes, lambda));

        // l1C


        return newCuts;
    }

    private List<ICut> separateARCC(List<Route> routes, double[] lambda) {
        List<ICut> cuts = new ArrayList<>();
        int totalCustomer = Constant.TOTAL_CUSTOMER;
        int k = (int) Math.floor((double) Constant.TRUCK_PAYLOAD / Constant.DRONE_AND_EQUIPMENT_WEIGHT);

        for(int cust = 1; cust <= totalCustomer; cust++) {
            Set<Integer> C = new HashSet<>();
            C.add(cust);
            while(C.size() < totalCustomer) { // while C != V
                int best = -1;
                double bestViolation = -Double.MAX_VALUE;
                for(int candidate = 1; candidate <= totalCustomer; candidate++) {
                    if(C.contains(candidate)) continue;

                    Set<Integer> test = new HashSet<>(C);
                    test.add(candidate);
                    double vio = computeViolation(C, routes, lambda, k);
                    if(vio > bestViolation) {
                        bestViolation = vio;
                        best = candidate;
                    }

                }
                if(best == -1) break;
                C.add(best);

                double vio = computeViolation(C, routes, lambda, k);
                if(vio > VIOLATION_THRESHOLD) {
                    cuts.add(new ARCCut(C));
                    if(cuts.size() >= MAX_CUTS_PER_ROUND) return cuts;
                }
            }
        }

        return cuts;

    }

    private double computeViolation(Set<Integer> C, List<Route> routes, double[] lambda, int k) {
        double totalDemand = C.stream()
                            .mapToInt(c -> VRPInstance.nodes.get(c).demand)
                            .sum();
                
        double rhs = Math.ceil(totalDemand / Constant.DRONE_PAYLOAD - Constant.EPSILON);
        double lhs = 0;
        for(int r = 0; r < routes.size(); r++) {
            if(lambda[r] < Constant.EPSILON) continue; // route is not used in solution
            Route route = routes.get(r);
            int h = countEntries(C, route);
            lhs += lambda[r] * h * (k - route.getNumDrone());
        }

        return rhs - lhs;
    }

    private int countEntries(Set<Integer> C, Route route) {

        int h_r = 0;

        for(int i = 1; i < route.sequence.size(); i++) {
            int pre = route.sequence.get(i-1).id;
            int cur = route.sequence.get(i).id;
            if(!C.contains(pre) && C.contains(cur)) h_r++;
        }

        for(var entry : route.customerDroneSchedule.entrySet()) {
            for(int c : entry.getValue().customerServed) {
                if(C.contains(c)) h_r++;
            }
        }

        return h_r;
    }

}
