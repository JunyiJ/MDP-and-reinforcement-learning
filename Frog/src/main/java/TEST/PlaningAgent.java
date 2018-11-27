package TEST;

import burlap.behavior.singleagent.Episode;
import burlap.mdp.singleagent.environment.Environment;

public interface PlaningAgent {
    Episode runLearningEpisode(Environment env);

    Episode runLearningEpisode(Environment env, int maxSteps);
}
