import java.math.BigInteger;
import java.util.ArrayList;
import java.util.List;

import com.gurobi.gurobi.*;

public class MasterProblem {

    private GRBEnv env;
    private GRBModel model;

    private GRBConstr[] coverConstr;
    private GRBConstr   truckConstr;
    private GRBConstr   droneConstr;
    public  List<GRBConstr> cutsConstr;

    private List<GRBVar> artificialVars;
    private List<GRBVar> realVars;
    private List<Route>  realVarRoutes;

    public double[] artificialValues;
    public double   objectiveValue;

    public MasterProblem(int totalCustomer) throws GRBException {
        env = new GRBEnv(true);
        env.set(GRB.IntParam.LogToConsole, 0);
        env.set("LogFile", "gurobi.log");
        env.start();
        model = new GRBModel(env);

        coverConstr = new GRBConstr[totalCustomer];
        for (int i = 0; i < totalCustomer; i++) {
            coverConstr[i] = model.addConstr(new GRBLinExpr(),
                                             GRB.EQUAL, 1.0, "cover_" + (i + 1));
        }
        truckConstr = model.addConstr(new GRBLinExpr(), GRB.LESS_EQUAL,
                                      Constant.MAX_VEHICLE, "c_truck");
        droneConstr = model.addConstr(new GRBLinExpr(), GRB.LESS_EQUAL,
                                      Constant.MAX_DRONE,   "c_drone");

        artificialVars   = new ArrayList<>();
        realVars         = new ArrayList<>();
        realVarRoutes    = new ArrayList<>();
        cutsConstr       = new ArrayList<>();
        artificialValues = new double[totalCustomer];

        initializeArtificialColumns(totalCustomer);
        model.update();
    }

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

    /** Reduce the RHS of the two resource constraints (for branch-and-price). */
    public void setResourceCaps(int vehCap, int droneCap) throws GRBException {
        truckConstr.set(GRB.DoubleAttr.RHS, vehCap);
        droneConstr.set(GRB.DoubleAttr.RHS, droneCap);
        model.update();
    }

    /**
     * For every customer in `forcedCustomers`, replace the coverage row
     * `Σ λ_r + a_c = 1` with `Σ λ_r + a_c ≤ 0`. This forces both real
     * columns and the artificial to be zero on that customer, which is
     * exactly what we want: the customer is already served by a forced route.
     */
    public void setUncoveredMask(BigInteger forcedCustomers) throws GRBException {
        for (int i = 0; i < coverConstr.length; i++) {
            if (forcedCustomers.testBit(i + 1)) {
                coverConstr[i].set(GRB.CharAttr.Sense, GRB.LESS_EQUAL);
                coverConstr[i].set(GRB.DoubleAttr.RHS, 0.0);
            }
        }
        model.update();
    }

    public void addColumn(Route route, CuttingPlanes cuttingPlanes) throws GRBException {
        double cost = route.totalTime;
        GRBColumn col = new GRBColumn();

        for (int customer : route.customerServed) {
            if (customer == 0 || customer > Constant.TOTAL_CUSTOMER) continue;
            col.addTerm(1, coverConstr[customer - 1]);
        }
        col.addTerm(1, truckConstr);
        col.addTerm(route.getNumDrone(), droneConstr);

        List<ICut> cuts = cuttingPlanes.cuts;
        for (int i = 0; i < cuts.size(); i++) {
            double coef = cuts.get(i).getCoefficientForRoute(route);
            if (Math.abs(coef) > Constant.EPSILON) col.addTerm(coef, cutsConstr.get(i));
        }

        GRBVar realVar = model.addVar(0, 1, cost, GRB.CONTINUOUS, col,
                                      "real_" + realVars.size());
        realVars.add(realVar);
        realVarRoutes.add(route);
        model.update();
    }

    public void addCut(ICut cut) throws GRBException {
        GRBLinExpr lhs = new GRBLinExpr();
        char sense = (cut instanceof ARCCut) ? GRB.GREATER_EQUAL
                  : (cut instanceof R1Cut)  ? GRB.LESS_EQUAL
                  : (char) 0;
        if (sense == 0) throw new IllegalArgumentException("Unknown cut type");

        GRBConstr constr = model.addConstr(lhs, sense, cut.getRHS(),
                                           "cut_" + cutsConstr.size());
        cutsConstr.add(constr);

        for (int i = 0; i < realVars.size(); i++) {
            double coef = cut.getCoefficientForRoute(realVarRoutes.get(i));
            if (Math.abs(coef) > Constant.EPSILON)
                model.chgCoeff(constr, realVars.get(i), coef);
        }
        model.update();
    }

    public void solve() throws GRBException {
        model.set(GRB.IntParam.Method, 1);
        model.optimize();

        int status = model.get(GRB.IntAttr.Status);
        if (status == GRB.Status.OPTIMAL) {
            objectiveValue   = model.get(GRB.DoubleAttr.ObjVal);
            artificialValues = extractArtificialVariableValues();
        } else if (status == GRB.Status.INFEASIBLE) {
            throw new GRBException("Master LP infeasible", status);
        } else {
            throw new GRBException("Master LP status " + status, status);
        }
    }

    public double[] extractArtificialVariableValues() throws GRBException {
        double[] v = new double[artificialVars.size()];
        for (int i = 0; i < artificialVars.size(); i++)
            v[i] = artificialVars.get(i).get(GRB.DoubleAttr.X);
        return v;
    }

    public double[] getDuals() throws GRBException {
        double[] pi = new double[coverConstr.length];
        for (int i = 0; i < coverConstr.length; i++)
            pi[i] = coverConstr[i].get(GRB.DoubleAttr.Pi);
        return pi;
    }

    public double[] getPrimes() throws GRBException {
        double[] lambda = new double[realVars.size()];
        for (int i = 0; i < realVars.size(); i++)
            lambda[i] = realVars.get(i).get(GRB.DoubleAttr.X);
        return lambda;
    }

    public double getDualVehicle() throws GRBException {
        return truckConstr.get(GRB.DoubleAttr.Pi);
    }
    public double getDualDrone() throws GRBException {
        return droneConstr.get(GRB.DoubleAttr.Pi);
    }

    public List<Route> getRealRoutes() { return realVarRoutes; }
    public double getObjectiveValue() { return objectiveValue; }

    public void dispose() throws GRBException {
        model.dispose();
        env.dispose();
    }
}