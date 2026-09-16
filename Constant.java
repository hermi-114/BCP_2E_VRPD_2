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
    public static double TRUCK_SPEED = 35;
    public static double TRUCK_PAYLOAD; // based on dataset, get in capacities.txt

    //
    public static double DRONE_WEIGHT = 11.0; // kg
    public static double DRONE_AND_EQUIPMENT_WEIGHT; // 20% truck capacity
    public static double DRONE_PAYLOAD = 10.0; // kg
    public static double DRONE_BATTERY_WEIGHT = 9.25; // kg
    public static double DRONE_SPINNING_BLADE_AREA = 0.586; // m^2 per blade
    public static int DRONE_BLADE_NUMBER = 4; // blades
    public static double DRONE_BATTERY_CAPACITY = 2308.8; // Wh
    public static double DRONE_SPEED = 35; // km/h
    public static double DRONE_SETUP_TIME = 0.083; // h

    public static double G_FORCE = 9.81; // N/kg
    public static double AIR_DENSITY = 1.204; // kg/m^3

    public static final double EPSILON = 1e-7;
    public static final double ROUTE_REDUCED_COST_THRESHOLD = -1e-2;
    public static final double M = 1e7 - 1;

}
