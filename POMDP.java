import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Random;

public class POMDP {
	
	private static DecimalFormat df2 = new DecimalFormat("#.##");
	
	public static int [][]obstacles = {
			{3, 1},
			{3, 2},
			{4, 3}
	};
	
	public static int [][]goal = {{2,4}};
	public static int []agent = new int[2];
	public static int [][]bound_x = {{1,0},{2,0},{3,0},{4,0},{5,0},{1,6},{2,6},{3,6},{4,6},{5,6}};
	public static int [][]bound_y = {{0,1},{0,2},{0,3},{0, 4},{0, 5},{6, 1},{6, 2},{6, 3},{6, 4},{6, 5}};
	
	//public static double [][]beliefValues = new double[5][5];
	public static double beliefProb = (double)1/22;

	public static List<Coordinates> no_action= new ArrayList<Coordinates>();
	public static List<Coordinates> bound = new ArrayList<Coordinates>();
	
	public static double gamma = 0.9;
	//public static int iteration = 100;
	
	public static boolean isConverged = false;
	
	public static Map<Integer, Double> reward = new HashMap<Integer, Double>();
	public static Map<Integer, Double> belief = new HashMap<Integer, Double>();
	public static Map<Integer, String> direction = new HashMap<Integer, String>();
	
	public static List<Coordinates> updated = new ArrayList<Coordinates>();
	public static List<Coordinates> present = new ArrayList<Coordinates>();
	
	public static int counter = 0;
	
	public static double p = 0.8;
	public static double q = 0.1;
	
	public static double pCorrectObs = 0.8;
	public static double pDeviateObs = 0.1;
	
	public static void assignRandomAgent(){
		Random random = new Random();
		agent[0]=random.nextInt(5)+1;
		agent[1]=random.nextInt(5)+1;
		boolean flag = false;
		//
		for(int i=0; i< obstacles.length;++i){
			if(obstacles[i][0] == agent[0] && obstacles[i][1] == agent[1]){
				flag = true;
				break;
			}
		}
		if(flag)
			assignRandomAgent();
	}
	
	public static double calculateWithBelief(double sum){
		double calculate = 0.0;
		Iterator<Integer> itr = belief.keySet().iterator();
		for (; itr.hasNext(); ) {
			int index = itr.next();
			calculate += sum*belief.get(index);
		}
		return calculate;
	}
	
	
	/**
	 * Method to calculate vertical sum
	 * @param next_list
	 * @param m
	 * @param isNorth
	 * @return
	 */
	public static double goVertical(List<Coordinates> next_list, Coordinates m, boolean isNorth){
		double sum_correct = 0.0, sum_neigh_E = 0.0, sum_neigh_W = 0.0;
		Coordinates next_co = null;
		
		if(isNorth){
			next_co = new Coordinates(m.x-1, m.y);
		} else {
			next_co = new Coordinates(m.x+1, m.y);
		}
		
		if(!bound.contains(next_co)){
			int next_pos = 5 * (next_co.x-1) + next_co.y;
			sum_correct = gamma * (p * reward.get(next_pos));
			if(!next_list.contains(next_co) && !no_action.contains(next_co)){
				next_list.add(next_co);
			}
		} else {
			sum_correct = 0;
		}
		//Neighbouring cell west
		next_co = new Coordinates(m.x, m.y-1);
		if(!bound.contains(next_co)){
			int next_pos = 5 * (next_co.x-1) + next_co.y;
			sum_neigh_W = gamma * (q * reward.get(next_pos));
			if(!next_list.contains(next_co) && !no_action.contains(next_co)){
				next_list.add(next_co);
			}
		} else {
			sum_neigh_W = 0;
		}
		//Neighbouring cell East
		next_co = new Coordinates(m.x, m.y+1);
		if(!bound.contains(next_co)){
			int next_pos = 5 * (next_co.x-1) + next_co.y;
			sum_neigh_E = gamma * (q * reward.get(next_pos));
			if(!next_list.contains(next_co) && !no_action.contains(next_co)){
				next_list.add(next_co);
			}
		} else {
			sum_neigh_E = 0;
		}
		
		return calculateWithBelief(sum_correct+sum_neigh_E+sum_neigh_W);
	}
	
	/**
	 * Method to calculate horizontal sum
	 * @param next_list
	 * @param m
	 * @param isEast
	 * @return
	 */
	public static double goHorizontal(List<Coordinates> next_list, Coordinates m, boolean isEast){
		double sum_correct = 0.0, sum_neigh_N = 0.0, sum_neigh_S = 0.0;
		Coordinates next_co = null;
		
		if(isEast){
			next_co = new Coordinates(m.x, m.y+1);
		} else {
			next_co = new Coordinates(m.x, m.y-1);
		}
		if(!bound.contains(next_co)){
			int index = 5*(next_co.x-1)+next_co.y;
			sum_correct = gamma * (p * reward.get(index));
			if(!next_list.contains(next_co) && !no_action.contains(next_co)){
				next_list.add(next_co);
			}
		} else {
			sum_correct = 0;
		}
		
		next_co = new Coordinates(m.x+1, m.y);
		if(!bound.contains(next_co)){
			int index = 5*(next_co.x-1)+next_co.y;
			sum_neigh_N = gamma * (q * reward.get(index));
			if(!next_list.contains(next_co) && !no_action.contains(next_co)){
				next_list.add(next_co);
			}
		} else {
			sum_neigh_N = 0;
		}
		next_co = new Coordinates(m.x-1, m.y);
		if(!bound.contains(next_co)){
			int index = 5*(next_co.x-1)+next_co.y;
			sum_neigh_S = gamma * (q * reward.get(index));
			if(!next_list.contains(next_co) && !no_action.contains(next_co)){
				next_list.add(next_co);
			}
		} else {
			sum_neigh_S = 0;
		}
		
		return calculateWithBelief(sum_correct+sum_neigh_N+sum_neigh_S);
	}
	
	static{
		for(int i=0; i<bound_x.length; ++i){
			bound.add(new Coordinates(bound_x[i][0], bound_x[i][1]));
			no_action.add(new Coordinates(bound_x[i][0], bound_x[i][1]));
		}
		for(int j = 0; j<bound_y.length; ++j){
			bound.add(new Coordinates(bound_y[j][0], bound_y[j][1]));
			no_action.add(new Coordinates(bound_y[j][0], bound_y[j][1]));
		}
		
		for(int j = 0; j < goal.length;++j ){
			no_action.add(new Coordinates(goal[j][0], goal[j][1]));
		}
		for(int j = 0; j<obstacles.length; ++j){
			no_action.add(new Coordinates(obstacles[j][0], obstacles[j][1]));
		}
		beliefProb = (double)1/(25 - obstacles.length);
		for (int i = 0; i < 5; i++) {
			for (int j = 0; j < 5; j++) {
				boolean flag = false;
				for(int[] obstacle : obstacles){
					if(obstacle[0] == i+1 && obstacle[1] == j+1){
						flag = true;
					}
				}
				if(!flag){
					int index = 5 * i + j;
					belief.put(index, beliefProb);
				}
			}
		}
		
		for(int i= 1; i < 26; ++i){
			reward.put(i, 1.0);
			//gridBeliefMap.put(i, 0.0);
			//direction.put(i, "0.0");
		}
		assignRandomAgent();
		//agent[0] = 1;
		//agent[1] = 4;
	}
	
	public static void getDir(){
		System.out.println("Route for converging to goal.");
		Iterator<Integer> directionItr = direction.keySet().iterator();
		System.out.println();
		int count = 0;
		for(;directionItr.hasNext();){
			int index = directionItr.next();
			System.out.print(direction.get(index)+" -> ");
			if(count > 8){
				System.out.println();
				count = 0;
			}
			++count;
		}
		System.out.print(" Goal");
	}
	
	public static Integer [] changeToObject(int [] arr){
		Integer []result = new Integer [arr.length];
		for(int i=0; i< arr.length; ++i){
			result[i] = new Integer(arr[i]);
		}
		return null;
	}
	
	public static void main(String[] args) {
		
		System.out.println("Agent starting position ["+agent[0]+", "+agent[1]+"]");
		
		System.out.println("Goal Position ["+goal[0][0]+", "+goal[0][1]+"]");
		String obstacle_str = "";
		for(int []obstacle : obstacles){
			obstacle_str += Arrays.toString(obstacle);
		}
		
		System.out.println("Obstacles "+obstacle_str);
		for(int i=0; i < obstacles.length; ++i){
			int index = 5 * (obstacles[i][0] - 1) + obstacles[i][1];
			reward.put(index, -100.0);
			//gridBeliefMap.put(index, -1.0);
			//direction.put(index, "-1.0");
		}
		
		for(int i=0; i < goal.length; ++i){
			int index = 5 * (goal[i][0] - 1) + goal[i][1];
			reward.put(index, 100.0);
			//gridBeliefMap.put(index, 1.0);
			//direction.put(index, "1.0");
		}
		updated.add(new Coordinates(agent[0], agent[1]));
		
		for(int i=0; true; ++i){
			List<Coordinates> next_list = new ArrayList<Coordinates>();
			for(Coordinates m : updated){
				Map<Character, Coordinates> next_arrow_dict = new HashMap<Character, Coordinates>();
				int pos = 5*(m.x -1) + m.y;
				present.add(m);
				
				Map<Character, Double> find_max = new HashMap<Character, Double>();
				
				//Go North
				Coordinates next_co = new Coordinates(m.x-1, m.y);
				next_arrow_dict.put('N', next_co);
				
				//Sum of going north
				double sum_go_north = goVertical(next_list,m, true);
				find_max.put('N', sum_go_north);
				
				
				//Go south
				next_co = new Coordinates(m.x+1, m.y);
				next_arrow_dict.put('S', next_co);
				
				double sum_go_south = goVertical(next_list, m, false);
				find_max.put('S', sum_go_south);
				
				
				//Go East
				next_co = new Coordinates(m.x, m.y+1);
				next_arrow_dict.put('E', next_co);
				
				
				double sum_go_east = goHorizontal(next_list, m, true);
				find_max.put('E', sum_go_east);
				
				//Go West 
				next_co = new Coordinates(m.x, m.y-1);
				next_arrow_dict.put('W', next_co);
				
				
				double sum_go_west = goHorizontal(next_list, m, false);
				find_max.put('W', sum_go_west);
				
				char max_direction = 'N';
				double max = Double.NEGATIVE_INFINITY;
				Iterator<Character> mapItr = find_max.keySet().iterator();
				for(;mapItr.hasNext();){
					char ch = mapItr.next();
					if(max < find_max.get(ch)){
						max_direction = ch;
						max = find_max.get(ch);
					}
				}
				
				Map<Character, Double> cango = new HashMap<Character, Double>();
				Coordinates coordinate = next_arrow_dict.get('N');
				if(!bound.contains(coordinate)){
					cango.put('N', sum_go_north);
				}
				coordinate = next_arrow_dict.get('S');
				if(!bound.contains(coordinate)){
					cango.put('S', sum_go_south);
				}
				coordinate = next_arrow_dict.get('E');
				if(!bound.contains(coordinate)){
					cango.put('E', sum_go_east);
				}
				coordinate = next_arrow_dict.get('W');
				if(!bound.contains(coordinate)){
					cango.put('W', sum_go_west);
				}
				
				max = Double.NEGATIVE_INFINITY;
				mapItr = cango.keySet().iterator();
				for(;mapItr.hasNext();){
					char ch = mapItr.next();
					if(max < cango.get(ch)){
						max_direction = ch;
						max = cango.get(ch);
					}
				}
				
				double max_val  = find_max.get(max_direction);
				max_val = reward.get(pos) + max_val;
				
				//Update Belief Map
				belief.put(5*(m.x-1)+(m.y-1), max_val);
				normalizeBelief();
				Coordinates nextState = null, comingFrom1 = null, comingFrom2 = null;
				Coordinates deviationState1 = null, comingFrom1Deviation1 = null, comingFrom1Deviation2 = null, comingFrom2Deviation1 = null, comingFrom2Deviation2 = null, deviationState2 = null;
				//gridBeliefMap.put(pos, max_val);
				String arrow;
				if(max_direction == 'N'){
					//arrow = '\u2191';
					arrow = "Up";
					nextState = new Coordinates(m.x-1, m.y);
					comingFrom1 = new Coordinates(m.x-1, m.y -1);
					comingFrom2 = new Coordinates(m.x-1, m.y +1);
					deviationState1 = new Coordinates(m.x, m.y - 1);	//west
					deviationState2 = new Coordinates(m.x, m.y + 1);	//East
					comingFrom1Deviation1 = new Coordinates(comingFrom1.x+1,comingFrom1.y);
					comingFrom1Deviation2 = new Coordinates(comingFrom1.x, comingFrom1.y-1);
					comingFrom2Deviation1 = new Coordinates(comingFrom2.x+1, comingFrom2.y);
					comingFrom2Deviation2 = new Coordinates(comingFrom2.x, comingFrom2.y+1);
				} else if(max_direction == 'S'){
					//arrow = '\u2193';
					arrow = "Down";
					comingFrom1 = new Coordinates(m.x+1, m.y -1);
					comingFrom2 = new Coordinates(m.x+1, m.y +1);
					deviationState1 = new Coordinates(m.x, m.y - 1);
					deviationState2 = new Coordinates(m.x, m.y + 1);
					comingFrom1Deviation1 = new Coordinates(comingFrom1.x-1, comingFrom1.y);
					comingFrom1Deviation2 = new Coordinates(comingFrom1.x, comingFrom1.y-1);
					
					comingFrom2Deviation1 = new Coordinates(comingFrom2.x-1, comingFrom2.y);
					comingFrom2Deviation2 = new Coordinates(comingFrom2.x, comingFrom2.y+1);
					
					nextState = new Coordinates(m.x+1,  m.y);
				} else if(max_direction == 'E'){
					//arrow = '\u2192';
					arrow = "Right";
					comingFrom1 = new Coordinates(m.x-1, m.y +1);
					comingFrom2 = new Coordinates(m.x+1, m.y +1);
					deviationState1 = new Coordinates(m.x-1, m.y);
					deviationState2 = new Coordinates(m.x+1, m.y);
					comingFrom1Deviation1 = new Coordinates(comingFrom1.x, comingFrom1.y-1);
					comingFrom1Deviation2 = new Coordinates(comingFrom1.x-1, comingFrom1.y);
					
					comingFrom2Deviation1 = new Coordinates(comingFrom2.x, comingFrom2.y-1);
					comingFrom2Deviation2 = new Coordinates(comingFrom2.x+1, comingFrom2.y);
					nextState = new Coordinates(m.x, m.y+1);
				} else {
					//arrow = '\u2190';
					arrow = "Left";
					comingFrom1 = new Coordinates(m.x-1, m.y -1);
					comingFrom2 = new Coordinates(m.x+1, m.y -1);
					deviationState1 = new Coordinates(m.x-1, m.y);
					deviationState2 = new Coordinates(m.x+1, m.y);
					comingFrom1Deviation1 = new Coordinates(comingFrom1.x, comingFrom1.y+1);
					comingFrom1Deviation2 = new Coordinates(comingFrom1.x-1, comingFrom1.y);
					
					comingFrom2Deviation1 = new Coordinates(comingFrom2.x, comingFrom2.y+1);
					comingFrom2Deviation2 = new Coordinates(comingFrom2.x+1, comingFrom2.y);
					nextState = new Coordinates(m.x, m.y - 1);
				}
				
				//Recalculate Belief
				
				double belief1 = 0, belief2 = 0, belief3 = 0;
				int belief1Index = 5*(nextState.x-1) + (nextState.y-1),
					belief2Index = 5*(comingFrom1.x-1) + (comingFrom1.y-1),
					belief3Index = 5*(comingFrom2.x-1) + (comingFrom2.y-1);
				int c1d1Index = 5*(comingFrom1Deviation1.x-1)+(comingFrom1Deviation1.y-1);
				int c1d2Index = 5*(comingFrom1Deviation2.x-1) + (comingFrom1Deviation2.y-1);
				int c2d1Index = 5*(comingFrom2Deviation1.x-1) + (comingFrom2Deviation1.y-1);
				int c2d2Index = 5*(comingFrom2Deviation2.x-1) + (comingFrom2Deviation2.y-1);
				int currentPosBeliefIndex = 5*(m.x-1)+(m.y-1);
				
				if(!no_action.contains(nextState)){
					belief1 = gamma * pCorrectObs * 
							(
									p * belief.get(currentPosBeliefIndex) + 
									q * (belief.get(belief2Index) != null ? belief.get(belief2Index) : 0) +
									q * (belief.get(belief3Index) != null ? belief.get(belief3Index) : 0)
							);
				}
				
				
				if(!no_action.contains(comingFrom1Deviation1)){
					belief2 = gamma * pDeviateObs * 
							(
									p * belief.get(c1d1Index)+
									q * (belief.get(belief1Index) != null ? belief.get(belief1Index) : 0)+
									q * (belief.get(c1d2Index) != null ? belief.get(c1d2Index) : 0)
							);
				}
				
				if(!no_action.contains(comingFrom2Deviation1)){
					belief3 = gamma * pDeviateObs * 
							(
									p * belief.get(c2d1Index) + 
									q * (belief.get(belief1Index) != null ? belief.get(belief1Index) : 0)+
									q * (belief.get(c2d2Index) != null ? belief.get(c2d2Index) : 0)
							);
				}
				
				double sum =0;
				if(!no_action.contains(nextState)){
					sum = belief1;
					//gridBeliefMap.put(belief1Index, belief1);
				}
				if(!no_action.contains(comingFrom1Deviation1)){
					sum += belief2;
					//gridBeliefMap.put(belief2Index, belief2);
				}
				if(!no_action.contains(comingFrom2Deviation1)){
					sum += belief3;
					//gridBeliefMap.put(belief3Index, belief3);
				}
				
				if(!no_action.contains(nextState)){
					belief.put(belief1Index, (double)(belief1/sum));
				}
				if(!no_action.contains(comingFrom1Deviation1)){
					sum += belief2;
					belief.put(belief2Index, (double)(belief2/sum));
				}
				if(!no_action.contains(comingFrom2Deviation1)){
					sum += belief3;
					belief.put(belief3Index, (double)(belief3/sum));
				}
				
				normalizeBelief();
				
				direction.put(counter++, arrow+"");
				
				if((belief.containsKey(belief1Index) && (belief.get(belief1Index)) >= 0.9 ) || (belief.containsKey(belief2Index) && (belief.get(belief2Index)) >= 0.9 )|| (belief.containsKey(belief3Index) && (belief.get(belief3Index)) >= 0.9 )){
				//if((belief.get(5*(m.x-1)+m.y-1) + 0.3) > 0.9){
					double max1 = Math.max(belief.get(belief1Index) != null ? belief.get(belief1Index) : 0, belief.get(belief2Index) != null ? belief.get(belief2Index) : 0);
					System.out.println("Belief of going from "+m+" to "+nextState + " is "+ (Math.max(belief.get(belief3Index) != null ? belief.get(belief3Index) : 0, max1))+ "  | Action : "+arrow);
					isConverged = true;
					break;
				} else {
					System.out.println("Belief of going from "+m+" to "+nextState + " is "+belief.get(5*(m.x-1)+m.y-1) + "  | Action : "+arrow);
				}
				
				
				
				//System.out.println(arrow);
				
				
				
			}
			if(isConverged)
				break;
			for(Coordinates coordinates : next_list){
				if(!updated.contains(coordinates)){
					updated.add(coordinates);
				}
			}
			Collections.reverse(updated);
			
		}
		System.out.println("Iteration : "+counter);
		System.out.println("Directions : ");
		getDir();
	}
	
	public static void normalizeBelief(){
		Iterator<Integer> itr = belief.keySet().iterator();
		double sum = 0;
		for(;itr.hasNext();){
			sum += belief.get(itr.next());
		}
		
		itr = belief.keySet().iterator();
		for(;itr.hasNext();){
			int index = itr.next();
			belief.put(index, (double)(belief.get(index)/sum));
		}
	}
	
	private static class Coordinates{
		int x;
		int y;
		public Coordinates(int x, int y) {
			this.x = x;
			this.y = y;
		}
		
		@Override
		public boolean equals(Object obj) {
			return this.x == ((Coordinates)obj).x &&
					this.y == ((Coordinates)obj).y;
		}
		
		@Override
		public String toString() {
			return "{"+x+","+y+"}";
		}
	}
}
