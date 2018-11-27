package MLPJ4;

import burlap.mdp.auxiliary.DomainGenerator;
import burlap.mdp.core.StateTransitionProb;
import burlap.mdp.core.TerminalFunction;
import burlap.mdp.core.action.Action;
import burlap.mdp.core.action.UniversalActionType;
import burlap.mdp.core.state.State;
import burlap.mdp.singleagent.SADomain;
import burlap.mdp.singleagent.environment.SimulatedEnvironment;
import burlap.mdp.singleagent.model.FactoredModel;
import burlap.mdp.singleagent.model.RewardFunction;
import burlap.mdp.singleagent.model.statemodel.FullStateModel;
import burlap.shell.visual.VisualExplorer;
import burlap.visualizer.StatePainter;
import burlap.visualizer.StateRenderLayer;
import burlap.visualizer.Visualizer;

import java.awt.*;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.List;




public class Grid implements DomainGenerator{
    public static final String VAR_X = "x";
    public static final String VAR_Y = "y";

    public static final String ACTION_NORTH = "north";
    public static final String ACTION_SOUTH = "south";
    public static final String ACTION_EAST = "east";
    public static final String ACTION_WEST = "west";
    public static final String ACTION_NORTHEAST = "northeast";
    public static final String ACTION_SOUTHEAST = "southeast";
    public static final String ACTION_NORTHWEST = "northwest";
    public static final String ACTION_SOUTHWEST = "southwest";

    protected int goalx = 10;
    protected int goaly = 10;

    //ordered so first dimension is x
    protected int [][] map = new int[][]{
            {0,0,0,0,0,1,0,0,0,0,0},
            {0,0,0,0,0,0,0,0,0,0,0},
            {0,0,0,0,0,1,0,0,0,0,0},
            {0,0,0,0,0,1,0,0,0,0,0},
            {0,0,0,0,0,1,0,0,0,0,0},
            {1,0,1,1,1,1,1,1,0,1,1},
            {0,0,0,0,1,0,0,0,0,0,0},
            {0,0,0,0,1,0,0,0,0,0,0},
            {0,0,0,0,0,0,0,0,0,0,0},
            {0,0,0,0,1,0,0,0,0,0,0},
            {0,0,0,0,1,0,0,0,0,0,0},
    };

    public void setGoalLocation(int goalx, int goaly){
        this.goalx = goalx;
        this.goaly = goaly;
    }

    public SADomain generateDomain() {
        SADomain domain = new SADomain();
        domain.addActionTypes(
                new UniversalActionType(ACTION_NORTH),
                new UniversalActionType(ACTION_SOUTH),
                new UniversalActionType(ACTION_EAST),
                new UniversalActionType(ACTION_WEST),
                new UniversalActionType(ACTION_NORTHEAST),
                new UniversalActionType(ACTION_SOUTHEAST),
                new UniversalActionType(ACTION_NORTHWEST),
                new UniversalActionType(ACTION_SOUTHWEST)

        );
        GridWorldStateModel smodel = new GridWorldStateModel();
        RewardFunction rf = new ExampleRF(this.goalx, this.goaly);
        TerminalFunction tf = new ExampleTF(this.goalx, this.goaly);

        domain.setModel(new FactoredModel(smodel, rf, tf));

        return domain;
    }

    protected class GridWorldStateModel implements FullStateModel{

        protected double [][] transitionProbs;

        public GridWorldStateModel() {
            this.transitionProbs = new double[8][8];
            for(int i = 0; i < 8; i++){
                for(int j = 0; j < 8; j++){
                    double p = i != j ? 0.21/7 : 0.79;
                    transitionProbs[i][j] = p;
                }
            }
        }

        protected int actionDir(Action a){
            int adir = -1;
            if(a.actionName().equals(ACTION_NORTH)){
                adir = 0;
            }
            else if(a.actionName().equals(ACTION_SOUTH)){
                adir = 1;
            }
            else if(a.actionName().equals(ACTION_EAST)){
                adir = 2;
            }
            else if(a.actionName().equals(ACTION_WEST)){
                adir = 3;
            }
            else if(a.actionName().equals(ACTION_NORTHEAST)){
                adir = 4;
            }
            else if(a.actionName().equals(ACTION_SOUTHEAST)){
                adir = 5;
            }
            else if(a.actionName().equals(ACTION_NORTHWEST)){
                adir = 6;
            }
            else if(a.actionName().equals(ACTION_SOUTHWEST)){
                adir = 7;
            }
            return adir;
        }

        protected int [] moveResult(int curX, int curY, int direction){

            //first get change in x and y from direction using 0: north; 1: south; 2:east; 3: west
            // 4: northeast; 5: southeast; 6: northwest; 7:southwest;
            int xdelta = 0;
            int ydelta = 0;
            if(direction == 0){
                ydelta = 1;
            }
            else if(direction == 1){
                ydelta = -1;
            }
            else if(direction == 2){
                xdelta = 1;
            }
            else if(direction == 3){
                xdelta = -1;
            }
            else if(direction == 4){
                xdelta = 1;
                ydelta = 1;
            }
            else if(direction == 5){
                xdelta = 1;
                ydelta = -1;
            }
            else if(direction == 6){
                xdelta = -1;
                ydelta = 1;
            }
            else if(direction == 7){
                xdelta = -1;
                ydelta = -1;
            }

            int nx = curX + xdelta;
            int ny = curY + ydelta;

            int width = Grid.this.map.length;
            int height = Grid.this.map[0].length;

            //make sure new position is valid (not a wall or off bounds)
            if(nx < 0 || nx >= width || ny < 0 || ny >= height ||
                    Grid.this.map[nx][ny] == 1){
                nx = curX;
                ny = curY;
            }

            return new int[]{nx,ny};

        }

        @Override
        public List<StateTransitionProb> stateTransitions(State s, Action a) {
            //get agent current position
            GridState gs = (GridState) s;
            int curX = gs.x;
            int curY = gs.y;

            int adir = actionDir(a);
            List<StateTransitionProb> tps =  new ArrayList<StateTransitionProb>(8);
            StateTransitionProb noChange = null;
            for(int i=0; i<8; i++){
                int [] newPos = this.moveResult(curX, curY, i);
                if(newPos[0] != curX || newPos[1] != curY) {
                    //new possible outcome
                    GridState ns = gs.copy();
                    ns.x = newPos[0];
                    ns.y = newPos[1];

                    //create transition probablity object and add to our list of outcomes
                    tps.add(new StateTransitionProb(ns, this.transitionProbs[adir][i]));
                }
                else{
                    if(noChange != null){
                        noChange.p += this.transitionProbs[adir][i];
                    }
                    else{
                        noChange = new StateTransitionProb(s.copy(), this.transitionProbs[adir][i]);
                        tps.add(noChange);
                    }
                }
            }
            return tps;
        }

        @Override
        public State sample(State s, Action a) {
            return null;
        }

    }

    public static class ExampleTF implements TerminalFunction {

        int goalX;
        int goalY;

        public ExampleTF(int goalX, int goalY) {
            this.goalX = goalX;
            this.goalY = goalY;
        }

        @Override
        public boolean isTerminal(State s) {

            //get location of agent in next state
            int ax = (Integer) s.get(VAR_X);
            int ay = (Integer) s.get(VAR_Y);

            //are they at goal location?
            if (ax == this.goalX && ay == this.goalY) {
                return true;
            }

            return false;
        }
    }

    public static class ExampleRF implements RewardFunction {
        int goalX;
        int goalY;

        public ExampleRF(int goalX, int goalY) {
            this.goalX = goalX;
            this.goalY = goalY;
        }

        @Override
        public double reward(State s, Action a, State sprime){
            int ax = (Integer)s.get(VAR_X);
            int ay = (Integer)s.get(VAR_Y);
            if (ax == this.goalX && ay == this.goalY){
                return 100.;
            }
            return -1;
        }
    }

    public class WallPainter implements StatePainter {

        public void paint(Graphics2D g2, State s, float cWidth, float cHeight) {

            //walls will be filled in black
            g2.setColor(Color.BLACK);

            //set up floats for the width and height of our domain
            float fWidth = Grid.this.map.length;
            float fHeight = Grid.this.map[0].length;

            //determine the width of a single cell
            //on our canvas such that the whole map can be painted
            float width = cWidth / fWidth;
            float height = cHeight / fHeight;

            //pass through each cell of our map and if it's a wall, paint a black rectangle on our
            //cavas of dimension widthxheight
            for(int i = 0; i < Grid.this.map.length; i++){
                for(int j = 0; j < Grid.this.map[0].length; j++){

                    //is there a wall here?
                    if(Grid.this.map[i][j] == 1){

                        //left coordinate of cell on our canvas
                        float rx = i*width;

                        //top coordinate of cell on our canvas
                        //coordinate system adjustment because the java canvas
                        //origin is in the top left instead of the bottom right
                        float ry = cHeight - height - j*height;

                        //paint the rectangle
                        g2.fill(new Rectangle2D.Float(rx, ry, width, height));

                    }


                }
            }

        }


    }


    public class AgentPainter implements StatePainter {


        @Override
        public void paint(Graphics2D g2, State s,
                          float cWidth, float cHeight) {

            //agent will be filled in gray
            g2.setColor(Color.GRAY);

            //set up floats for the width and height of our domain
            float fWidth = Grid.this.map.length;
            float fHeight = Grid.this.map[0].length;

            //determine the width of a single cell on our canvas
            //such that the whole map can be painted
            float width = cWidth / fWidth;
            float height = cHeight / fHeight;

            int ax = (Integer)s.get(VAR_X);
            int ay = (Integer)s.get(VAR_Y);

            //left coordinate of cell on our canvas
            float rx = ax*width;

            //top coordinate of cell on our canvas
            //coordinate system adjustment because the java canvas
            //origin is in the top left instead of the bottom right
            float ry = cHeight - height - ay*height;

            //paint the rectangle
            g2.fill(new Ellipse2D.Float(rx, ry, width, height));

        }

    }

    public StateRenderLayer getStateRenderLayer(){
        StateRenderLayer rl = new StateRenderLayer();
        rl.addStatePainter(new Grid.WallPainter());
        rl.addStatePainter(new Grid.AgentPainter());


        return rl;
    }

    public Visualizer getVisualizer(){
        return new Visualizer(this.getStateRenderLayer());
    }


    public static void main(String [] args){
        Grid gen = new Grid();
        gen.setGoalLocation(10, 10);
        SADomain domain = gen.generateDomain();
        State initialState = new GridState(0, 0);
        SimulatedEnvironment env = new SimulatedEnvironment(domain, initialState);

        Visualizer v = gen.getVisualizer();
        VisualExplorer exp = new VisualExplorer(domain, env, v);

        exp.addKeyAction("w", ACTION_NORTH, "");
        exp.addKeyAction("s", ACTION_SOUTH, "");
        exp.addKeyAction("d", ACTION_EAST, "");
        exp.addKeyAction("a", ACTION_WEST, "");
        exp.addKeyAction("e", ACTION_NORTHEAST, "");
        exp.addKeyAction("c", ACTION_SOUTHEAST, "");
        exp.addKeyAction("z", ACTION_SOUTHWEST, "");
        exp.addKeyAction("q", ACTION_NORTHWEST, "");

        exp.initGUI();
    }
}
