import com.gurobi.gurobi.GRBException;

public class ColumnGeneration {
    
    private final int maxLoopTime = 1000; // for test
    // public static final Set<String> existedSequences = new HashSet<>();

    public void solve() throws GRBException {
        MasterProblem master = new MasterProblem(Constant.TOTAL_CUSTOMER);
        PricingProblem pricing = new PricingProblem();

        int loopTime = 0;
        while (true && (loopTime++ < maxLoopTime)) {
        // while(true) {

            master.solve();

            double[] pi = master.getDualVariables();
            Route bestRoute = pricing.findBestRoute(pi, Constant.MAX_DRONE_PER_VEHICLE);

            if(bestRoute == null) {
                break;
            }

            master.addRealColumn(bestRoute);
            
            VRPInstance.routePool.add(bestRoute);
        }
        
        // get all dummies remaining in the routePool
        // if one found, the customer is cannot be served
        
        double[] dummyVals = master.artificialValues;
        master.dispose();

        for(var val : dummyVals) {
            if(val > Constant.EPSILON) {
                System.out.println("Infeasible! There is customer cannot be served");
                System.out.println(val);
            }
        }

        System.out.println("Generated " + VRPInstance.routePool.size() + " real routes");
    }
}
