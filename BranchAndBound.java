import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Stack;

public class BranchAndBound {
    private BBNode bestNode = null;
    private Solution bestSolution = null;
    private double minTime = Double.MAX_VALUE;
    private Map<Integer, Route> routeMap = new HashMap<>();
    List<List<Integer>> customerRoutes;
    
    
    public BranchAndBound() {
        rewriteRoutesId();
        mapping();
    } 

    
    private void rewriteRoutesId() {
        int id = 0;
        for(Route r : VRPInstance.routePool) {
            r.id = id++;
        }
    }

    private void mapping() {
        for(Route r : VRPInstance.routePool) {
            routeMap.put(r.id, r);
        }
    }

    // private void routesForCustomer() {
    //     customerRoutes = new ArrayList<>();
    //     customerRoutes.add(Collections.emptyList());

    //     for(int cust = 1; cust <= Constant.TOTAL_CUSTOMER; cust++) {
    //         List<Integer> selectedRoutes = new ArrayList<>();
    //         for(Route r : VRPInstance.routePool) {
    //             if(r.customerServedHashed.testBit(cust)) {
    //                 selectedRoutes.add(r.id);
    //             }
    //         }
    //         customerRoutes.add(selectedRoutes);
    //     }
    // }

    // private void sort() {
    //     // TODO
    //     for(int cust = 1; cust <= Constant.TOTAL_CUSTOMER; cust++) {
    //         Collections.sort(customerRoutes.get(cust), Comparator.comparingDouble(r -> routeMap.get(r).totalTime));
    //     }
        
    // }

    public void run() {

        System.out.println("=================== Branch and bound ====================");

        // routesForCustomer();
        // sort();

        Stack<Solution> solStack = new Stack<>();
        Stack<Integer>  idxStack = new Stack<>();
        solStack.push(new Solution());
        idxStack.push(0);

        int nRoutes = routeMap.size();

        while (!solStack.isEmpty()) {
            Solution sol = solStack.pop();
            int nextIdx  = idxStack.pop();

            // --- leaf: all routes decided ---
            if (nextIdx >= nRoutes) {
                if (sol.routes.isEmpty()) continue;         // empty is trivially invalid
                ConstraintChecker check = new ConstraintChecker(sol);
                check.checkAll();
                if (check.isValid() && sol.objectiveValue() < minTime) {
                    bestSolution = sol;
                    minTime = sol.objectiveValue;
                }
                continue;
            }

            // --- optional pruning (see section below) ---
            // if (sol.routes.size() > Constant.MAX_VEHICLE) continue;

            Route r = routeMap.get(nextIdx);

            // Branch A — skip r
            solStack.push(sol);
            idxStack.push(nextIdx + 1);

            // Branch B — take r
            List<Route> newRoutes = new ArrayList<>(sol.routes);
            newRoutes.add(r);
            solStack.push(new Solution(newRoutes));
            idxStack.push(nextIdx + 1);
        }

        ConstraintChecker checker = new ConstraintChecker(bestSolution);
        checker.checkAll();
        if(!checker.isValid()) bestSolution = null;

    }

    public Solution getSolution() { return bestSolution; }
}
