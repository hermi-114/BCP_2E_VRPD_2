import java.util.ArrayList;
import java.util.List;
import com.gurobi.gurobi.*;
import java.awt.color.ICC_ColorSpace;
import java.lang.reflect.Array;


public class MasterProblem {

    private GRBEnv env;
    private GRBModel model;
    private GRBConstr[] coverConstr;
    private GRBConstr truckConstr;
    private GRBConstr droneConstr;
    public List<GRBConstr> cutsConstr;
    private List<GRBVar> artificialVars;
    private List<GRBVar> realVars;

    private GRBLinExpr objectiveFunc;

    public double[] artificialValues;
    public double objectiveValue;
    

    public MasterProblem(int totalCustomer) throws GRBException {

        env = new GRBEnv(true);
        env.set(GRB.IntParam.LogToConsole, 0);
        env.set("LogFile", "gurobi.log");
        env.start();
        model = new GRBModel(env);

        objectiveFunc = new GRBLinExpr();

        artificialValues = new double[totalCustomer];
        cutsConstr = new ArrayList<>();

        coverConstr = new GRBConstr[totalCustomer];
        for(int i = 0; i < totalCustomer; i++) {
            coverConstr[i] = model.addConstr(new GRBLinExpr(), GRB.EQUAL, 1.0, "c_" + (i+1));
        }
        truckConstr = model.addConstr(new GRBLinExpr(), GRB.LESS_EQUAL, Constant.MAX_VEHICLE, "c_truck");
        droneConstr = model.addConstr(new GRBLinExpr(), GRB.LESS_EQUAL, Constant.MAX_DRONE, "c_drone");

        artificialVars = new ArrayList<>();
        realVars = new ArrayList<>();

        initializeBigMColumns(totalCustomer);

        model.update();

    }

    private void initializeBigMColumns(int totalCustomer) throws GRBException {
        double M = 9999.0;

        for(int i = 0; i < totalCustomer; i++) {

            GRBColumn col = new GRBColumn();
            col.addTerm(1, coverConstr[i]);

            GRBVar dummy = model.addVar(0, 1, M, GRB.CONTINUOUS, col, "dummy_" + i);

            artificialVars.add(dummy);
            objectiveFunc.addTerm(M, dummy);
        }

        model.update();
    }

    public void addColumn(Route route, CuttingPlanes cuttingPlanes) throws GRBException {
        // System.out.println("Master.addColumn: customerServed = " + route.customerServed);

        double cost = route.totalTime;
        GRBColumn col = new GRBColumn();
        
        for(int customer : route.customerServed) {
            if (customer == 0) continue;
            col.addTerm(1, coverConstr[customer-1]); // constraint in [0, totalCustomer)
        }
        col.addTerm(1, truckConstr);
        col.addTerm(route.getNumDrone(), droneConstr);


        List<ICut> cuts = cuttingPlanes.cuts;
        
        if (route.customerServed.isEmpty()) {
            System.out.println("WARNING: customerServed is empty! Route: " + route);
}
        for(int i = 0; i < cuts.size(); i++) {
            double coef = cuts.get(i).getCoefficientForRoute(route);
            if(Math.abs(coef) > Constant.EPSILON) {
                col.addTerm(coef, cutsConstr.get(i));
            }
        }
        

        GRBVar realVar = model.addVar(0, 1, cost, GRB.CONTINUOUS, col, "real_" + realVars.size());
        
        realVars.add(realVar);
        objectiveFunc.addTerm(cost, realVar);
        
        model.update();
    }

    public double[] extractArtificialVariableValues() throws GRBException {
        double[] values = new double[artificialVars.size()];

        for(int i = 0; i < artificialVars.size(); i++) {
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

        for(int i = 0; i < realVars.size(); i++) {
            lambda[i] = realVars.get(i).get(GRB.DoubleAttr.X);
        }

        return lambda;
    }

    public void solve() throws GRBException {

        model.set(GRB.IntParam.Method, 1);
        model.optimize();

        if(Config.PRINT_SWITCH_CMD) System.out.println("Solving...");

        int status = model.get(GRB.IntAttr.Status);

        if(status == GRB.Status.OPTIMAL) {
            if(Config.PRINT_SWITCH_CMD) {
                System.out.println("Model is optimal\n");
                this.objectiveValue = model.get(GRB.DoubleAttr.ObjVal);
                // System.out.println(this.objectiveValue);
            }

            
            artificialValues = extractArtificialVariableValues();
        } else if (status == GRB.Status.INFEASIBLE) {
            throw new GRBException("Master LP is infeasible! ", status);

        } else {
            throw new GRBException("Master LP failed with status " + status, status);
        }
    }

    public void dispose() throws GRBException {
        model.dispose();
        env.dispose();
    }

    public double getDualVehicle() throws GRBException { return truckConstr.get(GRB.DoubleAttr.Pi); }
    public double getDualDrone() throws GRBException { return droneConstr.get(GRB.DoubleAttr.Pi); }

    
    public void addCut(ICut cut) throws GRBException {
        GRBLinExpr lhs = new GRBLinExpr();
        char sense;
        if (cut instanceof ARCCut) {
            sense = GRB.GREATER_EQUAL;
        } else if (cut instanceof R1Cut) {
            sense = GRB.LESS_EQUAL;
        } else {
            throw new IllegalArgumentException("Unknown cut type");
        }
        GRBConstr constr = model.addConstr(lhs, sense, cut.getRHS(), "cut_" + cutsConstr.size());
        cutsConstr.add(constr);
        model.update();

    }

    public GRBConstr getCutConstraint(int index) { return cutsConstr.get(index); }
    
}
