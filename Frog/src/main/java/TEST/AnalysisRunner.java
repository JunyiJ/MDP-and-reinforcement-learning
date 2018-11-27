package TEST;


import burlap.behavior.policy.Policy;
import burlap.behavior.policy.PolicyUtils;
import burlap.behavior.singleagent.auxiliary.StateReachability;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.ValueFunctionVisualizerGUI;
import burlap.behavior.singleagent.learning.tdmethods.QLearning;
import burlap.behavior.singleagent.planning.stochastic.policyiteration.PolicyIteration;
import burlap.behavior.valuefunction.ValueFunction;
import burlap.domain.singleagent.gridworld.GridWorldDomain;
import burlap.mdp.core.Domain;
import burlap.mdp.core.TerminalFunction;
import burlap.mdp.core.state.State;
import burlap.mdp.singleagent.SADomain;
import burlap.mdp.singleagent.environment.SimulatedEnvironment;
import burlap.mdp.singleagent.model.RewardFunction;
import burlap.mdp.singleagent.oo.OOSADomain;
import burlap.statehashing.simple.SimpleHashableStateFactory;
import burlap.mdp.singleagent.SADomain;
import burlap.behavior.singleagent.Episode;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;

import java.util.List;

public class AnalysisRunner {

    final SimpleHashableStateFactory hashingFactory = new SimpleHashableStateFactory();
//set up the state hashing system
//	final hashingFactory = new DiscreteStateHashFactory();
//		hashingFactory.setAttributesForClass(GridWorldDomain.CLASSAGENT,
//			domain.getObjectClass(GridWorldDomain.CLASSAGENT).attributeList);

    private int MAX_ITERATIONS;
    private int NUM_INTERVALS;
    public int maxX;
    public int maxY;

    public AnalysisRunner(int MAX_ITERATIONS, int NUM_INTERVALS, int maxX, int maxY){
        this.MAX_ITERATIONS = MAX_ITERATIONS;
        this.NUM_INTERVALS = NUM_INTERVALS;
        this.maxX = maxX;
        this.maxY = maxY;

        int increment = MAX_ITERATIONS/NUM_INTERVALS;
        for(int numIterations = increment;numIterations<=MAX_ITERATIONS;numIterations+=increment ){
            AnalysisAggregator.addNumberOfIterations(numIterations);

        }

    }
    public void runValueIteration(ExampleGridWorld gen, SADomain domain,
                                  State initialState, double Gamma) {
        System.out.println("//Value Iteration Analysis//");
        ValueIteration vi = null;
        Policy p = null;
        Episode ea = null;
        List<State> states = null;
        int increment = MAX_ITERATIONS/NUM_INTERVALS;
        for(int numIterations = increment;numIterations<=MAX_ITERATIONS;numIterations+=increment ){
            long startTime = System.nanoTime();

            vi = new ValueIteration(
                    domain,
                    Gamma,
                    hashingFactory,
                    -1,
                    numIterations); //Added a very high delta number in order to guarantee that value iteration occurs the max number of iterations
            //for comparison with the other algorithms.

            // run planning from our initial state
            p = vi.planFromState(initialState);
            AnalysisAggregator.addMillisecondsToFinishValueIteration((int) (System.nanoTime()-startTime)/1000000);

            // evaluate the policy with one roll out visualize the trajectory
            ea = PolicyUtils.rollout(p, initialState, domain.getModel());
            states = ea.stateSequence;
            AnalysisAggregator.addValueIterationReward(calcRewardInEpisode(ea));
            AnalysisAggregator.addStepsToFinishValueIteration(ea.numTimeSteps());
        }

//		Visualizer v = gen.getVisualizer();
//		new EpisodeSequenceVisualizer(v, domain, Arrays.asList(ea));
        AnalysisAggregator.printValueIterationResults();
        AnalysisAggregator.printValueIterationTimeResults();
        AnalysisAggregator.printValueIterationRewards();
        System.out.println("\n\n");
    }

    public void runPolicyIteration(ExampleGridWorld gen, SADomain domain,
                                  State initialState, double Gamma) {
        System.out.println("//Policy Iteration Analysis//");
        PolicyIteration pi = null;
        Policy p = null;
        Episode ea = null;
        List<State> states = null;
        int increment = MAX_ITERATIONS/NUM_INTERVALS;
        for(int numIterations = increment;numIterations<=MAX_ITERATIONS;numIterations+=increment ){
            long startTime = System.nanoTime();

            pi = new PolicyIteration(
                    domain,
                    Gamma,
                    hashingFactory,
                    -1,
                    numIterations,
                    numIterations); //Added a very high delta number in order to guarantee that value iteration occurs the max number of iterations
            //for comparison with the other algorithms.

            // run planning from our initial state
            p = pi.planFromState(initialState);
            AnalysisAggregator.addMillisecondsToFinishPolicyIteration((int) (System.nanoTime()-startTime)/1000000);

            // evaluate the policy with one roll out visualize the trajectory
            ea = PolicyUtils.rollout(p, initialState, domain.getModel());
            states = ea.stateSequence;
            AnalysisAggregator.addPolicyIterationReward(calcRewardInEpisode(ea));
            AnalysisAggregator.addStepsToFinishPolicyIteration(ea.numTimeSteps());
        }

//		Visualizer v = gen.getVisualizer();
//		new EpisodeSequenceVisualizer(v, domain, Arrays.asList(ea));
        AnalysisAggregator.printPolicyIterationResults();
        AnalysisAggregator.printPolicyIterationTimeResults();
        AnalysisAggregator.printPolicyIterationRewards();
        System.out.println("\n\n");
    }

    public void runQLearning(ExampleGridWorld gen, OOSADomain domain,
                             State initialState,
                             SimulatedEnvironment env) {
        System.out.println("//Q Learning Analysis//");

        QLearning agent = null;
        Policy p = null;
        Episode ea = null;
        int increment = MAX_ITERATIONS/NUM_INTERVALS;
        for(int numIterations = increment;numIterations<=MAX_ITERATIONS;numIterations+=increment ){
            long startTime = System.nanoTime();

            agent = new QLearning(
                    domain,
                    0.99,
                    hashingFactory,
                    0.99, 0.99);

            for (int i = 0; i < numIterations; i++) {
                ea = agent.runLearningEpisode(env);
                env.resetEnvironment();
            }

            p = agent.planFromState(initialState);
            AnalysisAggregator.addQLearningReward(calcRewardInEpisode(ea));
            AnalysisAggregator.addMillisecondsToFinishQLearning((int) (System.nanoTime()-startTime)/1000000);
            AnalysisAggregator.addStepsToFinishQLearning(ea.numTimeSteps());

        }

    }



    public void simpleValueFunctionVis(ValueFunction valueFunction, Policy p, List<State> allStates, int maxX, int maxY){
//		ValueFunctionVisualizerGUI gui = GridWorldDomain.getGridWorldValueFunctionVisualization( allStates, 11, 11, valueFunction, p);
        ValueFunctionVisualizerGUI gui = GridWorldDomain.getGridWorldValueFunctionVisualization(allStates, maxX, maxY, valueFunction, p);
        gui.initGUI();

    }





    public double calcRewardInEpisode(Episode ea) {
        double myRewards = 0;

        //sum all rewards
        for (int i = 0; i<ea.rewardSequence.size(); i++) {
            myRewards += ea.rewardSequence.get(i);
        }
        return myRewards;
    }

}