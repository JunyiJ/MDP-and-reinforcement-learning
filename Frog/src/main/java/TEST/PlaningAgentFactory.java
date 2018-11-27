package TEST;

public interface PlaningAgentFactory {
    public String getAgentName();

    /**
     * Generates a new LearningAgent object and returns it.
     * @return a LearningAgent object.
     */
    public PlaningAgent generateAgent();
}
