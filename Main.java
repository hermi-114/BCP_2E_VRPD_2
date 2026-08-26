import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.math.BigInteger;
import java.util.ArrayList;
import java.util.List;

public class Main {
    public static void main(String[] args) {

        
        String fileName = "C101.txt";
        String fileOutName = "output.txt";
        boolean printSwitch = false;

        DataLoader.load("./data/Solomon/" + fileName);

        long startTime = System.currentTimeMillis();

        VRPInstance.calculateDistance();

        DroneScheduleEnumeration droneSchedulesEnum = new DroneScheduleEnumeration();
        droneSchedulesEnum.solve();

        long endTime = System.currentTimeMillis();

        if(printSwitch) {
            printParetoFront("./output/" + fileOutName);
            // printParetoFront(10);
        }

        
        System.out.printf("Progamme runs in %ds\n", (endTime-startTime)/1000);        
        
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
