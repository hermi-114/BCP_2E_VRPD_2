import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;


public class DataLoader {
    
    public static int truckNum;
    public static int droneNum;
    public static int nodeNum;
    public static String instanceName;
    public static Map<Integer, Route> routeMap;


    public static void loadCustomer(String path) {
        System.out.println("Start reading file...");

        int counter = 0;
        List<Integer> customers = new ArrayList<>();

        try (BufferedReader br = new BufferedReader(new FileReader(path))) {
    
            String line;
    
            while((line=br.readLine()) != null && counter <= Config.SIZE_CUSTOMER_DATASET) {

                if (line.trim().isEmpty()) {
                    continue;
                }

                String[] attrs = line.trim().split("\\s+");

                int id = Integer.parseInt(attrs[0]) - 1;
                double xCoor = Double.parseDouble(attrs[1]);
                double yCoor = Double.parseDouble(attrs[2]);
                int demand = (int)Double.parseDouble(attrs[3]);
                double readyTime = Double.parseDouble(attrs[4]) / 60;
                double deadline = Double.parseDouble(attrs[5]) / 60;
                double truckServiceTime = Double.parseDouble(attrs[6]) / 60; // m -> h

                Node node = new Node(id, xCoor, yCoor, demand, readyTime, deadline, truckServiceTime);

                VRPInstance.nodes.add(node);

                if(id != 0) customers.add(id);

                counter++;

            }
            
            if(counter == 0) {
                System.err.println("counter = 0");
                return;
            }

            Constant.TOTAL_CUSTOMER = counter - 1; // customers and 1 depot(id=0)
            Constant.TRUCK_MAX_SHIFT_TIME = VRPInstance.nodes.get(0).tw_b;

            int numCannotServeredByDrone = Constant.TOTAL_CUSTOMER / 5;
            Collections.shuffle(customers);
            for(int i = 0; i < numCannotServeredByDrone; i++) {
                Node node = VRPInstance.nodes.get(customers.get(i));
                if(node.servingTime < (node.tw_b - node.tw_a)) node.canServedByDrone = false; // if it cannot be served ny truck, it must be served by drone
            }

            for (int c = 1; c <= Constant.TOTAL_CUSTOMER; c++) {
                Node node = VRPInstance.nodes.get(c);
                if (node.demand > Constant.DRONE_PAYLOAD || !node.canServedByDrone)
                    VRPInstance.truckOnlyMask = VRPInstance.truckOnlyMask.setBit(c);
            }

            br.close();
            System.out.println("Done read file");
            System.out.println();

            System.out.printf("[Constant] %-32s = %s%n", "TRUCK_PAYLOAD", Constant.TRUCK_PAYLOAD);
            System.out.printf("[Constant] %-32s = %s%n", "DRONE_AND_EQUIPMENT_WEIGHT", Constant.DRONE_AND_EQUIPMENT_WEIGHT);
            System.out.printf("[Constant] %-32s = %s%n", "k = floor(Q / q_d)", Constant.MAX_DRONE_PER_VEHICLE);
            // --- new lines ---------------------------------------------------------
            System.out.printf("[Constant] %-32s = %s%n", "DRONE_SPEED", Constant.DRONE_SPEED);
            System.out.printf("[Constant] %-32s = %s%n", "DRONE_PAYLOAD", Constant.DRONE_PAYLOAD);
            System.out.printf("[Constant] %-32s = %s%n", "MAX_DRONE", Constant.MAX_DRONE);
            // ----------------------------------------------------------------------
            
            
        } catch(FileNotFoundException e) {
            System.err.println("FILE NOT FOUND: " + '"' + path + '"');
        } catch (Exception e) {
            System.err.println("LOAD DATA ERROR: " + e.getMessage());
        }

    }

    public static void loadFleet(String dataset) {
        try(BufferedReader br = new BufferedReader(new FileReader("./data/Solomon/capacities.txt"))) {
            
            String name = dataset.replace(".txt", "");
            int idx = 0;
            while(name.charAt(idx) == 'R' || name.charAt(idx) == 'C') idx++;
            String type = name.substring(0, idx);
            String set = name.substring(idx);
            String line;

            while((line = br.readLine()) != null) {
                String[] attr = line.split(":");
                if(attr[0].equals(type) && attr[1].compareTo(set) < 0) {
                    Constant.TRUCK_PAYLOAD = Double.parseDouble(attr[2]);
                    Constant.DRONE_AND_EQUIPMENT_WEIGHT = Constant.TRUCK_PAYLOAD / 5;
                } 
            }

            br.close();

        } catch(IOException e) {
            System.err.println("capacities.txt NOT FOUND");
        }
    }
}
