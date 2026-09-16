import java.util.ArrayList;
import java.util.List;

import com.gurobi.gurobi.*;

/**
 * Restricted master problem (RMP) for the column generation loop.
 *
 * Structure:
 *
 *   min   sum_r  c_r  * x_r     +  sum_i  M * a_i
 *   s.t.
 *       sum_r  [j in r]  x_r  +  a_j  =  1         for every customer j
 *       sum_r            x_r  <=  MAX_VEHICLE
 *       sum_r d_r *      x_r  <=  MAX_DRONE
 *       (cuts)
 *       x, a  >= 0
 *
 * Conventions:
 *   - One "real" variable per route.  Its objective coefficient is route.totalTime.
 *   - One "artificial" (Big-M) variable per customer, used only if the route
 *     set cannot cover the customer.
 *   - Cuts are added on top; when a new cut is added after some columns already
 *     exist, the coefficients of those columns in the new cut must be updated
 *     (see `addCut`).
 */
public class MasterProblem {

    private GRBEnv env;
    private GRBModel model;

    // Constraint handles
    private GRBConstr[] coverConstr;      // one per customer: sum x = 1
    private GRBConstr truckConstr;        // sum x <= MAX_VEHICLE
    private GRBConstr droneConstr;        // sum d*x <= MAX_DRONE
    public  List<GRBConstr> cutsConstr;   // user-defined cuts

    // Variables
    private List<GRBVar> artificialVars;
    private List<GRBVar> realVars;
    // Parallel to realVars — needed to compute new cut coefficients when a cut
    // is added after columns already exist.
    private List<Route>  realVarRoutes;

    // Cached outputs
    public double[] artificialValues;     // values of artificial vars from last solve
    public double   objectiveValue;

    // =========================================================================
    // CONSTRUCTION
    // =========================================================================
    public MasterProblem(int totalCustomer) throws GRBException {

        env = new GRBEnv(true);
        env.set(GRB.IntParam.LogToConsole, 0);
        env.set("LogFile", "gurobi.log");
        env.start();
        model = new GRBModel(env);

        // --- cover constraints (one per customer) ---
        coverConstr = new GRBConstr[totalCustomer];
        for (int i = 0; i < totalCustomer; i++) {
            coverConstr[i] = model.addConstr(new GRBLinExpr(),
                                             GRB.EQUAL, 1.0, "cover_" + (i + 1));
        }

        // --- resource constraints ---
        truckConstr = model.addConstr(new GRBLinExpr(), GRB.LESS_EQUAL, Constant.MAX_VEHICLE, "c_truck");
        droneConstr = model.addConstr(new GRBLinExpr(), GRB.LESS_EQUAL, Constant.MAX_DRONE,   "c_drone");

        // --- variable lists ---
        artificialVars = new ArrayList<>();
        realVars       = new ArrayList<>();
        realVarRoutes  = new ArrayList<>();
        cutsConstr     = new ArrayList<>();
        artificialValues = new double[totalCustomer];

        initializeArtificialColumns(totalCustomer);

        model.update();
    }

    /**
     * Creates one artificial (Big-M) variable per customer.  Each covers
     * exactly one cover constraint with coefficient 1.
     *
     * The objective is set directly via the `obj` argument to addVar; the
     * model objective is the sum of these coefficients.
     */
    private void initializeArtificialColumns(int totalCustomer) throws GRBException {
        final double M = 9999.0;

        for (int i = 0; i < totalCustomer; i++) {
            GRBColumn col = new GRBColumn();
            col.addTerm(1, coverConstr[i]);

            GRBVar dummy = model.addVar(0, 1, M, GRB.CONTINUOUS, col, "dummy_" + i);
            artificialVars.add(dummy);
        }
        model.update();
    }

    // =========================================================================
    // COLUMN ADDITION
    // =========================================================================
    /**
     * Adds a route as a new column.
     *
     * Coefficients:
     *   - cover constraint j : 1 if route visits customer j
     *   - truckConstr        : 1 (every route uses one truck)
     *   - droneConstr        : route.getNumDrone()
     *   - each existing cut  : cut.getCoefficientForRoute(route)
     *
     * Objective coefficient: route.totalTime.
     */
    public void addColumn(Route route, CuttingPlanes cuttingPlanes) throws GRBException {

        double cost = route.totalTime;
        GRBColumn col = new GRBColumn();

        // cover constraints
        for (int customer : route.customerServed) {
            if (customer == 0 || customer > Constant.TOTAL_CUSTOMER) continue;
            col.addTerm(1, coverConstr[customer - 1]);
        }

        // resource constraints
        col.addTerm(1,                     truckConstr);
        col.addTerm(route.getNumDrone(),   droneConstr);

        // existing cuts
        List<ICut> cuts = cuttingPlanes.cuts;
        for (int i = 0; i < cuts.size(); i++) {
            double coef = cuts.get(i).getCoefficientForRoute(route);
            if (Math.abs(coef) > Constant.EPSILON) {
                col.addTerm(coef, cutsConstr.get(i));
            }
        }

        GRBVar realVar = model.addVar(0, 1, cost, GRB.CONTINUOUS, col,
                                      "real_" + realVars.size());
        realVars.add(realVar);
        realVarRoutes.add(route);

        model.update();
    }

    // =========================================================================
    // CUT ADDITION
    // =========================================================================
    /**
     * Adds a new cut.
     *
     * IMPORTANT: after the cut is created, we must also update the
     * coefficients of every existing real variable in this cut; otherwise the
     * LP is inconsistent with the intended cut for previously added columns.
     */
    public void addCut(ICut cut) throws GRBException {

        GRBLinExpr lhs = new GRBLinExpr();
        char sense;
        if (cut instanceof ARCCut) {
            sense = GRB.GREATER_EQUAL;
        } else if (cut instanceof R1Cut) {
            sense = GRB.LESS_EQUAL;
        } else {
            throw new IllegalArgumentException("Unknown cut type: " + cut.getClass());
        }

        GRBConstr constr = model.addConstr(lhs, sense, cut.getRHS(),
                                           "cut_" + cutsConstr.size());
        cutsConstr.add(constr);

        // --- update coefficients of existing real variables ---
        for (int i = 0; i < realVars.size(); i++) {
            Route route = realVarRoutes.get(i);
            double coef = cut.getCoefficientForRoute(route);
            if (Math.abs(coef) > Constant.EPSILON) {
                model.chgCoeff(constr, realVars.get(i), coef);
            }
        }

        model.update();
    }

    // =========================================================================
    // SOLVE / ACCESSORS
    // =========================================================================
    public void solve() throws GRBException {

        model.set(GRB.IntParam.Method, 1);          // dual simplex
        model.optimize();

        if (Config.PRINT_SWITCH_CMD) System.out.println("Master LP solved.");

        int status = model.get(GRB.IntAttr.Status);

        if (status == GRB.Status.OPTIMAL) {
            this.objectiveValue = model.get(GRB.DoubleAttr.ObjVal);
            artificialValues    = extractArtificialVariableValues();
            if (Config.PRINT_SWITCH_CMD) {
                System.out.println("Master obj = " + this.objectiveValue);
            }
        } else if (status == GRB.Status.INFEASIBLE) {
            throw new GRBException("Master LP is infeasible!", status);
        } else {
            throw new GRBException("Master LP failed with status " + status, status);
        }
    }

    public double[] extractArtificialVariableValues() throws GRBException {
        double[] values = new double[artificialVars.size()];
        for (int i = 0; i < artificialVars.size(); i++) {
            values[i] = artificialVars.get(i).get(GRB.DoubleAttr.X);
        }
        return values;
    }

    public double[] getDuals() throws GRBException {
        double[] pi = new double[coverConstr.length];
        for (int i = 0; i < coverConstr.length; i++) {
            pi[i] = coverConstr[i].get(GRB.DoubleAttr.Pi);
        }
        return pi;
    }

    public double[] getPrimes() throws GRBException {
        double[] lambda = new double[realVars.size()];
        for (int i = 0; i < realVars.size(); i++) {
            lambda[i] = realVars.get(i).get(GRB.DoubleAttr.X);
        }
        return lambda;
    }

    public double getDualVehicle() throws GRBException {
        return truckConstr.get(GRB.DoubleAttr.Pi);
    }

    public double getDualDrone() throws GRBException {
        return droneConstr.get(GRB.DoubleAttr.Pi);
    }

    public GRBConstr getCutConstraint(int index) { return cutsConstr.get(index); }

    public void dispose() throws GRBException {
        model.dispose();
        env.dispose();
    }

    /**
     * Removes all real columns whose current λ is below `threshold`.
     * Safe to call ONLY when the LP is optimal for the current column set.
     *
     * @return the number of columns removed.
     */
    public int pruneColumns(double threshold) throws GRBException {

        // 1. collect indices to remove
        List<Integer> toRemove = new ArrayList<>();
        for (int i = 0; i < realVars.size(); i++) {
            double val = realVars.get(i).get(GRB.DoubleAttr.X);
            if (val < threshold) toRemove.add(i);
        }
        if (toRemove.isEmpty()) return 0;

        // 2. remove from Gurobi, one at a time (your Gurobi API supports only singles)
        for (int k = toRemove.size() - 1; k >= 0; k--) {
            int idx = toRemove.get(k);
            model.remove(realVars.get(idx));
        }
        model.update();

        // 3. remove from our own lists, descending order
        for (int k = toRemove.size() - 1; k >= 0; k--) {
            int idx = toRemove.get(k);
            realVars.remove(idx);
            realVarRoutes.remove(idx);
        }

        return toRemove.size();
    }
    public List<Route> getRealRoutes() {
        return realVarRoutes;
    }
}