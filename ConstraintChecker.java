import java.util.ArrayList;
import java.util.List;

public class ConstraintChecker {

    private final int n = Constant.TOTAL_CUSTOMER;            // number of customers (nodes 1..n)
    private final int D = Constant.MAX_DRONE_PER_VEHICLE;     // maximum drones per vehicle
    private final int k_v = Constant.MAX_VEHICLE;             // available vehicles
    private final int k_d = Constant.MAX_DRONE;               // total drones available
    private final double[][] c_v = VRPInstance.distMatrix;    // travel time matrix, symmetric, size (n+1)x(n+1)
    
    private final Solution sol;
    private final List<Route> routes;

    private final int[][][] x;         // x[d][i][j] for i<j
    private final double[] y;          // y[d]
    private final int[][] z;           // z[d][i] (0/1)
    private final int[] u;             // u[d]

    private final List<String> errors = new ArrayList<>();

    public ConstraintChecker(Solution sol) {
        this.sol = sol;
        this.routes = sol.routes;

        this.x = new int[D+1][n+1][n+1];
        this.y = new double[D+1];
        this.z = new int[D+1][n+1];
        this.u = new int[D+1];
    }

    private void computeVariables() {
        if(routes == null || routes.isEmpty()) {
            errors.add("Null solution");
            return;
        }
        
        // 5: x
        for (Route r : routes) {
            int[][] b = new int[n + 1][n + 1];
            List<Node> seq = r.sequence;
            int size = seq.size();

            for (int cur = 1; cur < size; cur++) {
                int a = seq.get(cur - 1).id;
                int c = seq.get(cur).id;
                if (a == c) continue;
                int lo = Math.min(a, c), hi = Math.max(a, c);
                b[lo][hi]++;                       // count once
            }

            int d = r.getNumDrone();
            for (int i = 0; i <= n; i++)
                for (int j = i + 1; j <= n; j++)
                    x[d][i][j] += b[i][j];         // b already symmetric
        }

        // 6: y:= total time vehicle launches drone schedules on each node in routes in solution
        //        time on each node i:= droneSchedule(i).makespan, 
        //                          NOT max(droneSchedule(i).makespan, serve(i))
        for(Route r : routes) {
            int d = r.getNumDrone();
            for(int j = 1; j < r.sequence.size() - 1; j++) { // eg: route.sequence = 0 - 1 - 2 - 3 - 0
                Node customer = r.sequence.get(j);

                DroneSchedule schedule = r.customerDroneSchedule.getOrDefault(customer.id, null);
                if(schedule == null) continue;

                y[d] += schedule.makespan;
            }
        }

        // 7: z
        for(Route r : routes) {
            int d = r.getNumDrone();
            for(int j = 1; j < r.sequence.size() - 1; j++) { // eg: route.sequence = 0 - 1 - 2 - 3 - 0
                Node customer = r.sequence.get(j);

                DroneSchedule schedule = r.customerDroneSchedule.getOrDefault(customer.id, null);
                if(schedule == null) continue;

                for(List<Integer> sequence : schedule.sequences) {
                    for(int i : sequence) {
                        z[d][i]++;
                    }
                }
            }
        }

        // 8: u
        for(Route r : routes) {
            u[r.getNumDrone()]++;
        }

        // 9: lambdaR:= check if route with d drones is used in solution = {0,1}
        // -> loop through all routes in solution -> not used yet

    }

    // 1
    public void evaluateObjectiveValue() {
        double ans = 0;
        for (int d = 0; d <= D; d++) {                       // ← <= D
            for (int i = 0; i <= n; i++)
                for (int j = i + 1; j <= n; j++)             // ← i<j, avoid double count
                    ans += c_v[i][j] * x[d][i][j];
            ans += y[d];                                     // ← outside the i loop
        }
        sol.objectiveValue = ans;
    }

    // 2
    private void checkVisitEdgeOnce() {
        for(int i = 1; i <= n; i++) {

            double sum_to_i = 0;
            double sum_out_i = 0;
            double sum_drone_i = 0;

            for(int d = 0; d <= D; d++) {

                for(int j = 0; j < i; j++) {
                    sum_to_i += x[d][j][i];
                }

                for(int j = i+1; j <= n; j++) {
                    sum_out_i += x[d][i][j];
                }

                sum_drone_i += z[d][i];

            }

            double coverage = sum_to_i / 2 + sum_out_i / 2 + sum_drone_i;

            if (coverage < 1 - Constant.EPSILON) {
                errors.add("Customer_" + i + " is not visited at least once (coverage = " + coverage + ")");
            }

            if(Math.abs(coverage - 1) > Constant.EPSILON) {
                // errors.add("Conflict constraint (2) from customer_" + i);
            }
        }

    }

    // 3
    private void checkVehicleLimit() {
        int totalVehicle = 0;
        for(int d = 0; d <= D; d++) {
            totalVehicle += u[d];
        }

        if(totalVehicle > k_v) {
            errors.add("Total truck " + totalVehicle + " conflicts constraint (3) of truck limit " + k_v);
        }
    } 

    // 4
    private void checkDroneLimit() {
        int totalDrone = 0;
        for(int d = 0; d <= D; d++) {
            totalDrone += d * u[d];
        }

        if(totalDrone > k_d) {
            errors.add("Total drone " + totalDrone + " conflicts constraint (4) of drone limit " + k_d);
        }
    }

    public void checkAll() {
        computeVariables();
        evaluateObjectiveValue();
        checkVisitEdgeOnce();
        checkVehicleLimit();
        checkDroneLimit();

    }

    public List<String> getErrors() { return this.errors; }
    public boolean isValid() { return this.errors.isEmpty(); }
}
