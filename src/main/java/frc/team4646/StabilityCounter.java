package frc.team4646;

public class StabilityCounter {
  private final int threshold;
  private int stableCounts = 0;
  
  public StabilityCounter(int stableThreshold) {
    this.threshold = stableThreshold;
    reset();
  }

  public boolean calculate(boolean isCurrentlyStable) {
    stableCounts++;
    if (!isCurrentlyStable) {
      reset();
    }
    return isStable();
  }

  public void reset() {
    stableCounts = 0;
  }

  public int counts() {
    return stableCounts;
  }

  public boolean isStable() {
    return stableCounts >= threshold;
  }
}
