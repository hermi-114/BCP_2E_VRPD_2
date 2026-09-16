import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class BranchAndBound {
    private BBNode currentMinNode = null;
    private double currentMinTime = Double.MAX_VALUE;
    private Map<Integer, Route> routeMap = new HashMap<>();
    List<List<Integer>> customerRoutes;
    

    private void rewriteRoutesId() {
        int id = 0;
        for(Route r : VRPInstance.routePool) {
            r.id = id++;
            routeMap.put(r.id, r);
        }
    }

    private void mapping() {
        for(Route r : VRPInstance.routePool) {
            routeMap.put(r.id, r);
        }
    }

    private void routesForCustomer() {
        customerRoutes = new ArrayList<>();
        customerRoutes.add(Collections.emptyList());

        for(int cust = 1; cust <= Constant.TOTAL_CUSTOMER; cust++) {
            List<Integer> selectedRoutes = new ArrayList<>();
            for(Route r : VRPInstance.routePool) {
                if(r.customerServedHashed.testBit(cust)) {
                    selectedRoutes.add(r.id);
                }
            }
            customerRoutes.add(selectedRoutes);
        }
    }

    private void sort() {
        // TODO
        for(int cust = 1; cust <= Constant.TOTAL_CUSTOMER; cust++) {
            Collections.sort(customerRoutes.get(cust), Comparator.comparingDouble(r -> routeMap.get(r).totalTime));
        }
        
    }

    public BBNode travel(BBNode node, int currentLayer) {
        
        if(node.totalTime > currentMinTime) return null;

        for(int rId : customerRoutes.get(currentLayer + 1)) {
            Route route = routeMap.get(rId);
        }

        return null;
    }

    public boolean isDominated(BBNode node, Route route) {
        return false;

    }

    public void run() {
        rewriteRoutesId();
        mapping();
        routesForCustomer();
        sort();

    }

    public Solution constructSolution(BBNode leaf) {
        Solution sol = new Solution();

        for(int i = 0; i < routeMap.size(); i++) {
            if(leaf.selectedRoutes.testBit(i)) sol.addRoute(routeMap.get(i));
        }

        sol.objectiveValue();

        return sol;
    }
}
