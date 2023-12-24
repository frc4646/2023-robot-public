package frc.team4646;

public class DiagnosticState {
  public final LEDColor color;
  public final boolean flash;
  public final boolean critical;

  public DiagnosticState(LEDColor diagnostic) {
    this(diagnostic, false, false);
  }
  
  public DiagnosticState(LEDColor diagnostic, boolean critical) {
    this(diagnostic, critical, false);
  }

  public DiagnosticState(LEDColor diagnostic, boolean critical, boolean flash) {
    this.color = diagnostic;
    this.critical = critical;
    this.flash = flash;
  }
}
