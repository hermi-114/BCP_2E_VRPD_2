import com.gurobi.gurobi.*;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.math.BigInteger;
import java.util.ArrayList;
import java.util.List;

public class Main {
    public static void main(String[] args) {

        String fileOutName_drone = "drone.txt";
        String fileOutName_route = "route.txt";

        DataLoader.loadCustomer("./data/Solomon/" + (Config.INPUT_TYPE + Config.INPUT_SET + ".txt"));   // get customers' information
        DataLoader.loadFleet();                                                                         // get truck's capacity

        long startTime = System.currentTimeMillis();

        VRPInstance.calculateDistance(); // System.out.println(VRPInstance.nodes.get(0).toString());

        DroneScheduleEnumeration droneSchedulesEnum = new DroneScheduleEnumeration();
        droneSchedulesEnum.solve();

        long end_drone_enum = System.currentTimeMillis();

        ColumnGeneration columnGeneration = new ColumnGeneration();
        try {
            columnGeneration.solve();
            
        } catch (GRBException e) {
            e.printStackTrace();
        }

        long end_column_gen = System.currentTimeMillis();

        long endTime = System.currentTimeMillis();

        if(Config.PRINT_DRONE) {
            printParetoFront("./output/" + fileOutName_drone);
            // printParetoFront(10);
        }

        if(Config.PRINT_ROUTE) {
            printRoutePool("./output/" + fileOutName_route);
        }
        
        System.out.printf("\nDrone Schedules Enumeration: %ds\n", (end_drone_enum - startTime)/1000);
        System.out.printf("Column generation: %ds\n", (end_column_gen - end_drone_enum)/1000);
        
        System.out.printf("\nProgamme runs in %ds\n", (endTime - startTime)/1000);
        
    }

    public static void printRoutePool(String fileName) {
        PrintWriter out;

        try {
            out = new PrintWriter(fileName);

            for(var route : VRPInstance.routePool) {
                out.println(route);
            }

            out.close();

        } catch (FileNotFoundException e) {
            System.err.println("File not found " + e.getMessage());
        }
    }
    
    public static void printParetoFront(int customer) {

        System.out.println("\n==================================================== Customer " + customer + "  ====================================================");

        for(var S_d_i_K : DroneScheduleEnumeration.paretoMap.get(customer)) {
            for(int d = 1; d <= Constant.MAX_DRONE_PER_VEHICLE; d++) {
                for(var S_K : S_d_i_K.entrySet()) {
                    System.out.println(getCustomerServedSet(S_K.getKey()).toString());
                    for(DroneSchedule s : S_K.getValue().nonDominatedSchedules) {
                        System.out.println(s);
                    }
                    System.out.println();
                }
            }
        }

    }

    public static void printParetoFront(String fileName) {
        PrintWriter out;
        int counter = 0;

        try {
            out = new PrintWriter(fileName);

            for (int customer = 1; customer <= Constant.TOTAL_CUSTOMER; customer++) {

                // if(DroneScheduleEnumeration.paretoMap.size() <= customer) break;

                out.println("\n==================================================== Customer " + customer + "  ====================================================");
                for (int drone = 1; drone <= Constant.MAX_DRONE_PER_VEHICLE; drone++) {

                    for (var S_d_i_K : DroneScheduleEnumeration.paretoMap
                            .get(customer)
                            .get(drone)
                            .entrySet()) {

                        out.println("\n-" + customer + "- " + getCustomerServedSet(S_d_i_K.getKey()).toString() + " ---");

                        for (DroneSchedule schedule : S_d_i_K.getValue().nonDominatedSchedules) {
                            out.println(schedule.toString() + schedule.sequences.toString());
                            counter++;
                        }

                        out.println();
                    }
                }
                out.println("================================================================================================================== "  + customer + "\n");

            }

            System.out.println("TOTAL DRONE SCHEDULES: " + counter);

            out.close();
        } catch (FileNotFoundException e) {
            System.err.println("File not found " + e.getMessage());
        }
    }

    public static List<Integer> getCustomerServedSet(BigInteger customerServed) {
        List<Integer> indices = new ArrayList<>();

        BigInteger temp = customerServed;

        while (!temp.equals(BigInteger.ZERO)) {
            int index = temp.getLowestSetBit();
            indices.add(index);
            temp = temp.clearBit(index);
        }

        return indices;
    }

    
}
