package TEST;


import burlap.behavior.singleagent.auxiliary.EpisodeSequenceVisualizer;
import burlap.behavior.singleagent.auxiliary.StateReachability;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.LandmarkColorBlendInterpolation;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.PolicyGlyphPainter2D;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.StateValuePainter2D;
import burlap.behavior.singleagent.learning.tdmethods.SarsaLam;
import burlap.behavior.singleagent.planning.stochastic.policyiteration.PolicyIteration;
import burlap.domain.singleagent.gridworld.GridWorldTerminalFunction;
import burlap.domain.singleagent.gridworld.state.GridAgent;
import burlap.domain.singleagent.gridworld.state.GridLocation;
import burlap.domain.singleagent.gridworld.state.GridWorldState;
import burlap.mdp.auxiliary.DomainGenerator;
import burlap.mdp.auxiliary.common.SinglePFTF;
import burlap.mdp.auxiliary.stateconditiontest.StateConditionTest;
import burlap.mdp.auxiliary.stateconditiontest.TFGoalCondition;
import burlap.mdp.core.StateTransitionProb;
import burlap.mdp.core.TerminalFunction;
import burlap.mdp.core.action.Action;
import burlap.mdp.core.action.UniversalActionType;
import burlap.mdp.core.oo.OODomain;
import burlap.mdp.core.oo.propositional.PropositionalFunction;
import burlap.mdp.core.oo.state.OOState;
import burlap.mdp.core.oo.state.ObjectInstance;
import burlap.mdp.core.oo.state.generic.GenericOOState;
import burlap.mdp.core.state.State;
import burlap.mdp.core.state.vardomain.VariableDomain;
import burlap.mdp.singleagent.SADomain;
import burlap.mdp.singleagent.common.SingleGoalPFRF;
import burlap.mdp.singleagent.environment.SimulatedEnvironment;
import burlap.mdp.singleagent.model.FactoredModel;
import burlap.mdp.singleagent.model.RewardFunction;
import burlap.mdp.singleagent.model.statemodel.FullStateModel;
import burlap.mdp.singleagent.oo.OOSADomain;
import burlap.shell.visual.VisualExplorer;
import burlap.statehashing.HashableStateFactory;
import burlap.statehashing.simple.SimpleHashableStateFactory;
import burlap.visualizer.*;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import sun.awt.SunHints;
import burlap.behavior.policy.GreedyQPolicy;
import burlap.behavior.policy.Policy;
import burlap.behavior.policy.PolicyUtils;
import burlap.behavior.singleagent.Episode;
import burlap.behavior.singleagent.auxiliary.performance.LearningAlgorithmExperimenter;
import burlap.behavior.singleagent.auxiliary.performance.TrialMode;
import burlap.behavior.singleagent.learning.LearningAgent;
import burlap.behavior.singleagent.learning.LearningAgentFactory;
import burlap.behavior.singleagent.learning.tdmethods.QLearning;
import burlap.behavior.singleagent.auxiliary.performance.PerformanceMetric;
import burlap.behavior.valuefunction.ValueFunction;
import burlap.domain.singleagent.gridworld.GridWorldDomain;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.ValueFunctionVisualizerGUI;
import burlap.mdp.singleagent.common.GoalBasedRF;
import burlap.mdp.core.oo.state.generic.GenericOOState;

import java.awt.*;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;



/**
 * @author James MacGlashan.
 */
public class ExampleGridWorld implements DomainGenerator {

    public static final String VAR_X = "x";
    public static final String VAR_Y = "y";
    public static final String VAR_TYPE = "type";

    public static final String CLASS_AGENT = "agent";
    public static final String CLASS_LOCATION = "location";

    public static final String ACTION_FORWARD = "forward";
    public static final String ACTION_BACKWARD = "backward";
    public static final String ACTION_FLY = "fly";

    public static final String PF_AT = "at";


    private static Integer MAX_ITERATIONS = 20;
    private static Integer NUM_INTERVALS = 20;

    //ordered so first dimension is x
    protected static int [][] map = new int[][]{
            {0}, {0}, {1}, {0}, {1}, {0}, {0}, {0}, {1}, {0}, {1}, {0}, {1}, {0}, {0}, {0}, {1}, {0}, {0}, {0}
    };
    static int maxX = map.length;
    static int maxY = map[0].length;

    public List<PropositionalFunction> generatePfs(){
        return Arrays.<PropositionalFunction>asList(new AtLocation());
    }

    @Override
    public OOSADomain generateDomain() {

        OOSADomain domain = new OOSADomain();

        domain.addStateClass(CLASS_AGENT, ExGridAgent.class)
                .addStateClass(CLASS_LOCATION, EXGridLocation.class);


        domain.addActionTypes(
                new UniversalActionType(ACTION_FORWARD),
                new UniversalActionType(ACTION_BACKWARD),
                new UniversalActionType(ACTION_FLY)
        );

        OODomain.Helper.addPfsToDomain(domain, this.generatePfs());

        OOGridWorldStateModel smodel = new OOGridWorldStateModel();
        RewardFunction rf = new SingleGoalPFRF(domain.propFunction(PF_AT), 10, -1);
        TerminalFunction tf = new SinglePFTF(domain.propFunction(PF_AT));

        domain.setModel(new FactoredModel(smodel, rf, tf));


        return domain;

    }






    protected class OOGridWorldStateModel implements FullStateModel {


        protected double[][] transitionProbs;

        public OOGridWorldStateModel() {
            this.transitionProbs = new double[3][3];
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    double p = i != j ? 0.2/2 : 0.8;
                    transitionProbs[i][j] = p;
                }
            }
        }

        @Override
        public List<StateTransitionProb> stateTransitions(State s, Action a) {

            //get agent current position
            GenericOOState gs = (GenericOOState) s;
            ExGridAgent agent = (ExGridAgent) gs.object(CLASS_AGENT);

            int curX = agent.x;
            int curY = agent.y;

            int adir = actionDir(a);

            List<StateTransitionProb> tps = new ArrayList<StateTransitionProb>(3);
            StateTransitionProb noChange = null;
            for (int i = 0; i < 3; i++) {

                int[] newPos = this.moveResult(curX, curY, i);
                if (newPos[0] != curX || newPos[1] != curY) {
                    //new possible outcome
                    GenericOOState ns = gs.copy();
                    ExGridAgent nagent = (ExGridAgent) ns.touch(CLASS_AGENT);
                    nagent.x = newPos[0];
                    nagent.y = newPos[1];

                    //create transition probability object and add to our list of outcomes
                    tps.add(new StateTransitionProb(ns, this.transitionProbs[adir][i]));
                } else {
                    //this direction didn't lead anywhere new
                    //if there are existing possible directions
                    //that wouldn't lead anywhere, aggregate with them
                    if (noChange != null) {
                        noChange.p += this.transitionProbs[adir][i];
                    } else {
                        //otherwise create this new state and transition
                        noChange = new StateTransitionProb(s.copy(), this.transitionProbs[adir][i]);
                        tps.add(noChange);
                    }
                }

            }


            return tps;
        }

        @Override
        public State sample(State s, Action a) {

            s = s.copy();
            GenericOOState gs = (GenericOOState) s;
            ExGridAgent agent = (ExGridAgent) gs.touch(CLASS_AGENT);
            int curX = agent.x;
            int curY = agent.y;

            int adir = actionDir(a);

            //sample direction with random roll
            double r = Math.random();
            double sumProb = 0.;
            int dir = 0;
            for (int i = 0; i < 3; i++) {
                sumProb += this.transitionProbs[adir][i];
                if (r < sumProb) {
                    dir = i;
                    break; //found direction
                }
            }

            //get resulting position
            int[] newPos = this.moveResult(curX, curY, dir);

            //set the new position
            agent.x = newPos[0];
            agent.y = newPos[1];

            //return the state we just modified
            return gs;
        }

        protected int actionDir(Action a) {
            int adir = -1;
            if (a.actionName().equals(ACTION_FORWARD)) {
                adir = 0;
            } else if (a.actionName().equals(ACTION_BACKWARD)) {
                adir = 1;
            } else if (a.actionName().equals(ACTION_FLY)){
                adir = 2;
            }
            return adir;
        }


        protected int[] moveResult(int curX, int curY, int direction) {

            //first get change in x and y from direction using 0: north; 1: south; 2:east; 3: west
            int xdelta = 0;
            int ydelta = 0;
            if (direction == 0) {
                xdelta = 1;
            } else if (direction == 1) {
                xdelta = -1;
            } else if (direction == 2){
                xdelta = 2;
            }

            int nx = curX + xdelta;
            int ny = curY + ydelta;

            int width = ExampleGridWorld.this.map.length;
            int height = ExampleGridWorld.this.map[0].length;

            //make sure new position is valid (not a wall or off bounds)
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
                nx = curX;
                ny = curY;
            }
            if (ExampleGridWorld.this.map[nx][ny] == 1) {
                // If bump into wall, start from the beginning.
                nx = 0;
            }


            return new int[]{nx, ny};

        }
    }

        public StateRenderLayer getStateRenderLayer(){
            StateRenderLayer rl = new StateRenderLayer();
            rl.addStatePainter(new ExampleGridWorld.WallPainter());
            OOStatePainter ooStatePainter = new OOStatePainter();
            ooStatePainter.addObjectClassPainter(CLASS_LOCATION, new LocationPainter());
            ooStatePainter.addObjectClassPainter(CLASS_AGENT, new AgentPainter());
            rl.addStatePainter(ooStatePainter);


            return rl;
        }

        public Visualizer getVisualizer(){
            return new Visualizer(this.getStateRenderLayer());
        }


    protected class AtLocation extends PropositionalFunction {

        public AtLocation(){
            super(PF_AT, new String []{CLASS_AGENT, CLASS_LOCATION});
        }

        @Override
        public boolean isTrue(OOState s, String... params) {
            ObjectInstance agent = s.object(params[0]);
            ObjectInstance location = s.object(params[1]);

            int ax = (Integer)agent.get(VAR_X);
            int ay = (Integer)agent.get(VAR_Y);

            int lx = (Integer)location.get(VAR_X);
            int ly = (Integer)location.get(VAR_Y);

            return ax == lx && ay == ly;

        }

    }



    public class WallPainter implements StatePainter {

        public void paint(Graphics2D g2, State s, float cWidth, float cHeight) {

            //walls will be filled in black
            g2.setColor(Color.BLACK);

            //set up floats for the width and height of our domain
            float fWidth = ExampleGridWorld.this.map.length;
            float fHeight = ExampleGridWorld.this.map[0].length;

            //determine the width of a single cell
            //on our canvas such that the whole map can be painted
            float width = cWidth / fWidth;
            float height = cHeight / fHeight;

            //pass through each cell of our map and if it's a wall, paint a black rectangle on our
            //cavas of dimension widthxheight
            for(int i = 0; i < ExampleGridWorld.this.map.length; i++){
                for(int j = 0; j < ExampleGridWorld.this.map[0].length; j++){

                    //is there a wall here?
                    if(ExampleGridWorld.this.map[i][j] == 1){

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


    public class AgentPainter implements ObjectPainter {

        @Override
        public void paintObject(Graphics2D g2, OOState s, ObjectInstance ob,
                                float cWidth, float cHeight) {

            //agent will be filled in gray
            g2.setColor(Color.GRAY);

            //set up floats for the width and height of our domain
            float fWidth = ExampleGridWorld.this.map.length;
            float fHeight = ExampleGridWorld.this.map[0].length;

            //determine the width of a single cell on our canvas
            //such that the whole map can be painted
            float width = cWidth / fWidth;
            float height = cHeight / fHeight;

            int ax = (Integer)ob.get(VAR_X);
            int ay = (Integer)ob.get(VAR_Y);

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

    public class LocationPainter implements ObjectPainter {

        @Override
        public void paintObject(Graphics2D g2, OOState s, ObjectInstance ob,
                                float cWidth, float cHeight) {

            //agent will be filled in blue
            g2.setColor(Color.BLUE);

            //set up floats for the width and height of our domain
            float fWidth = ExampleGridWorld.this.map.length;
            float fHeight = ExampleGridWorld.this.map[0].length;

            //determine the width of a single cell on our canvas
            //such that the whole map can be painted
            float width = cWidth / fWidth;
            float height = cHeight / fHeight;

            int ax = (Integer)ob.get(VAR_X);
            int ay = (Integer)ob.get(VAR_Y);

            //left coordinate of cell on our canvas
            float rx = ax*width;

            //top coordinate of cell on our canvas
            //coordinate system adjustment because the java canvas
            //origin is in the top left instead of the bottom right
            float ry = cHeight - height - ay*height;

            //paint the rectangle
            g2.fill(new Rectangle2D.Float(rx, ry, width, height));


        }



    }

    public static double calcRewardInEpisode(Episode ea) {
        double myRewards = 0;

        //sum all rewards
        for (int i = 0; i<ea.rewardSequence.size(); i++) {
            myRewards += ea.rewardSequence.get(i);
        }
        return myRewards;
    }

    public static void manualValueFunctionVis(List<State> allStates, ValueFunction valueFunction, Policy p){
        //define color function
        LandmarkColorBlendInterpolation rb = new LandmarkColorBlendInterpolation();
        rb.addNextLandMark(0., Color.RED);
        rb.addNextLandMark(1., Color.BLUE);

        //define a 2D painter of state values, specifying
        //which variables correspond to the x and y coordinates of the canvas
        StateValuePainter2D svp = new StateValuePainter2D(rb);
        svp.setXYKeys("x", "y",
                new VariableDomain(0, maxX), new VariableDomain(0, maxY),
                1, 1);

        //create our ValueFunctionVisualizer that paints for all states
        //using the ValueFunction source and the state value painter we defined
        ValueFunctionVisualizerGUI gui = new ValueFunctionVisualizerGUI(allStates, svp, valueFunction);

        //define a policy painter that uses arrow glyphs for each of the grid world actions
        PolicyGlyphPainter2D spp = new PolicyGlyphPainter2D();
        spp.setXYKeys("x", "y",
                new VariableDomain(0, maxX), new VariableDomain(0, maxY),
                1, 1);

        spp.setActionNameGlyphPainter(ExampleGridWorld.ACTION_FORWARD, new ArrowActionGlyph(2));
        spp.setActionNameGlyphPainter(ExampleGridWorld.ACTION_BACKWARD, new ArrowActionGlyph(3));
        spp.setRenderStyle(PolicyGlyphPainter2D.PolicyGlyphRenderStyle.DISTSCALED);


        //add our policy renderer to it
        gui.setSpp(spp);
        gui.setPolicy(p);

        //set the background color for places where states are not rendered to grey
        gui.setBgColor(Color.GRAY);

        //start it
        gui.initGUI();
    }


    public static void main(String [] args){

        ExampleGridWorld gen = new ExampleGridWorld();
        final OOSADomain domain = gen.generateDomain();
        State initialState = new GenericOOState(new ExGridAgent(0, 0), new EXGridLocation(maxX-1, 0, "loc0"));
        SimulatedEnvironment env = new SimulatedEnvironment(domain, initialState);
        Visualizer v = gen.getVisualizer();
        VisualExplorer exp = new VisualExplorer(domain, env, v);
        exp.initGUI();

        long startTime = System.nanoTime();
        ValueIteration vi = new ValueIteration(domain, 0.99, new SimpleHashableStateFactory(),
                0.001, 50);
        GreedyQPolicy policy = vi.planFromState(initialState);
        AnalysisAggregator.addMillisecondsToFinishValueIteration((int) (System.nanoTime()-startTime)/1000000);
        Episode ea = PolicyUtils.rollout(policy, initialState, domain.getModel());
        AnalysisAggregator.addValueIterationReward(calcRewardInEpisode(ea));
        AnalysisAggregator.addStepsToFinishValueIteration(ea.numTimeSteps());

        List<State> states = ea.stateSequence;
        ArrayList<State> allStates = new ArrayList<State>();
        for(State s : states){
            GenericOOState gs = (GenericOOState) s;
            ExGridAgent agent = (ExGridAgent) gs.touch(CLASS_AGENT);
            allStates.add(new EXGridState(agent.x, agent.y));
            System.out.printf("%d, ", agent.get(VAR_X));
            System.out.printf("\n");
        }


//        List<State> allStates = StateReachability.getReachableStates(initialState,
//                domain, new SimpleHashableStateFactory());
//        manualValueFunctionVis(allStates, (ValueFunction)vi, policy);
//
        long startTime1 = System.nanoTime();
        PolicyIteration pi = new PolicyIteration(domain, 0.99, new SimpleHashableStateFactory(),
                0.001, 50, 50);
        GreedyQPolicy policy1 = pi.planFromState(initialState);
        AnalysisAggregator.addMillisecondsToFinishPolicyIteration((int) (System.nanoTime()-startTime1)/1000000);
        Episode ea1 = PolicyUtils.rollout(policy, initialState, domain.getModel());
        AnalysisAggregator.addPolicyIterationReward(calcRewardInEpisode(ea1));
        AnalysisAggregator.addStepsToFinishPolicyIteration(ea1.numTimeSteps());
//        manualValueFunctionVis(states, (ValueFunction)pi, policy1);
//
//
//
//        EpisodeSequenceVisualizer epv = new EpisodeSequenceVisualizer(v, domain, Arrays.asList(ea1));
//        epv.initGUI();
////        BufferedImage bi = new BufferedImage(200, 200, BufferedImage.TYPE_INT_ARGB);
////        SimulatedEnvironment env = new SimulatedEnvironment(domain, initialState);
////        VisualExplorer exp = new VisualExplorer(domain, env, v);
//
////        exp.addKeyAction("w", ACTION_NORTH, "");
////        exp.addKeyAction("s", ACTION_SOUTH, "");
////        exp.addKeyAction("d", ACTION_EAST, "");
////        exp.addKeyAction("a", ACTION_WEST, "");
////        exp.addKeyAction("e", ACTION_NORTHEAST, "");
////        exp.addKeyAction("c", ACTION_SOUTHEAST, "");
////        exp.addKeyAction("z", ACTION_SOUTHWEST, "");
////        exp.addKeyAction("q", ACTION_NORTHWEST, "");
////
////        exp.initGUI();
//
//
//
        AnalysisRunner runner = new AnalysisRunner(MAX_ITERATIONS,NUM_INTERVALS, maxX, maxY);
        runner.runValueIteration(gen,domain,initialState,0.99);
        runner.runPolicyIteration(gen,domain,initialState,0.99);
        LearningAgentFactory qLearningFactory = new LearningAgentFactory() {

            public String getAgentName() {
                return "Q-Learning";
            }


            public LearningAgent generateAgent() {
                return new QLearning(domain, 0.99, new SimpleHashableStateFactory(), 0.3, 0.1);
            }
        };

        LearningAgentFactory sarsaLearningFactory = new LearningAgentFactory() {

            public String getAgentName() {
                return "SARSA";
            }


            public LearningAgent generateAgent() {
                return new SarsaLam(domain, 0.99, new SimpleHashableStateFactory(), 0.0, 0.1, 1.);
            }
        };
        LearningAlgorithmExperimenter exp1 = new LearningAlgorithmExperimenter(env, 10, 100,
                qLearningFactory, sarsaLearningFactory);

        exp1.setUpPlottingConfiguration(500, 250, 2, 1000,
                TrialMode.MOST_RECENT_AND_AVERAGE,
                PerformanceMetric.CUMULATIVE_STEPS_PER_EPISODE,
                PerformanceMetric.AVERAGE_EPISODE_REWARD);

        exp1.startExperiment();
//        runner.runQLearning(gen, domain, initialState, env);
//
//
//        BasicBehavior learningRunner = new BasicBehavior();
//        learningRunner.runLearning();
    }


}