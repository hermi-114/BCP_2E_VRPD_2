public class Constant {

    public static int TOTAL_CUSTOMER;

    // ============== FLEET PARAMETERS ===================
    public static int MAX_VEHICLE = 10; 
    public static int MAX_DRONE = 40;
    public static int MAX_DRONE_PER_VEHICLE = 4;
    public static int MAX_NEIGHBOURS_PER_NEIGHBOURHOOD = 15; // customers
    public static int DRONE_MAX_STOP = 3; // maximum customers served by each drone from a node
    public static double TRUCK_MAX_SHIFT_TIME; // default: get the depot's deadline
    public static int MAX_SCHEDULE_IN_SDIK = 5;

    public static int LABEL_MAX_NG_SIZE = 10;

    // =============== CONSTANTS ===================
    public static double TRUCK_SPEED = 60;
    public static double TRUCK_PAYLOAD; // based on dataset, get in capacities.txt

    //
    public static double DRONE_WEIGHT = 11.0; // kg
    public static double DRONE_AND_EQUIPMENT_WEIGHT = 50; // default
    public static double DRONE_PAYLOAD = 10.0; // kg
    public static double DRONE_BATTERY_WEIGHT = 9.25; // kg
    public static double DRONE_SPINNING_BLADE_AREA = 0.586; // m^2 per blade
    public static int DRONE_BLADE_NUMBER = 4; // blades
    public static double DRONE_BATTERY_CAPACITY = 2308.8; // Wh
    public static double DRONE_SPEED = 60; // km/h
    public static double DRONE_SETUP_TIME = 0.083; // h

    public static double G_FORCE = 9.81; // N/kg
    public static double AIR_DENSITY = 1.204; // kg/m^3

    public static final double EPSILON = 1e-7;
    public static final double ROUTE_REDUCED_COST_THRESHOLD = -1e-2;
    public static final double M = 1e7 - 1;

    public static void initDerivedValues() {
        if (TRUCK_PAYLOAD <= 0) {
            throw new IllegalStateException(
                "TRUCK_PAYLOAD must be set by DataLoader.loadFleet() before initDerivedValues()");
        }
        DRONE_AND_EQUIPMENT_WEIGHT = 0.20 * TRUCK_PAYLOAD;

        int k = (int) Math.floor(TRUCK_PAYLOAD / DRONE_AND_EQUIPMENT_WEIGHT);
        System.out.println("[Constant] TRUCK_PAYLOAD             = " + TRUCK_PAYLOAD);
        System.out.println("[Constant] DRONE_AND_EQUIPMENT_WEIGHT = " + DRONE_AND_EQUIPMENT_WEIGHT);
        System.out.println("[Constant] k = floor(Q / q_d)         = " + k);
        if (k <= 0) {
            throw new IllegalStateException("k must be > 0; check TRUCK_PAYLOAD and DRONE_AND_EQUIPMENT_WEIGHT");
        }
    }

    // ============== THREE-STAGE CG / STABILISATION ===================
    public static int    MAX_COLUMNS_LIGHT      = 30;
    public static int    MAX_COLUMNS_HEURISTIC  = 30;
    public static int    MAX_COLUMNS_EXACT      = 150;

    public static int    ROUTE_ENUM_THRESHOLD   = 5000;    // Baldacci-style shortcut
    public static int    ROUTE_ENUM_HARD_CAP    = 200_000; // safety, in case of blow-up

    public static double SMOOTHING_ALPHA_INIT   = 0.9;     // Pessoa 2018
    public static double SMOOTHING_ALPHA_MIN    = 0.3;
    public static double SMOOTHING_ALPHA_DECAY  = 0.9;     // α ← α · DECAY when pricing fails

    public static int    ARC_ELIMINATION_START_ITER = 10;  // after this many CG iterations
    public static double ARC_ELIMINATION_EPS        = 1e-6;

    // ============== STRONG BRANCHING (Pecin 2017a) ===================
    public static int    STRONG_BRANCHING_PHASE1_KEEP   = 12;   // candidates after phase 1
    public static int    STRONG_BRANCHING_PHASE2_KEEP   = 6;    // candidates after phase 2
    public static int    STRONG_BRANCHING_HEUR_CG_ROUNDS = 3;   // heuristic CG rounds in phase 3
    public static double STRONG_BRANCHING_HISTORY_DECAY = 0.9;   // decay past history scores
    public static double STRONG_BRANCHING_INFEAS_SCORE  = 1e6;  // score when a child is infeasible
    public static int    MAX_ARC_CANDIDATES_PER_NODE    = 200;   // cap on arc candidates

}
