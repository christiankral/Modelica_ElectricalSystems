within PowerFlow;
package Interfaces
  connector Terminal "General power terminal"
    replaceable package PhaseSystem = PhaseSystems.PartialPhaseSystem
      "Phase system" 
      annotation (choicesAllMatching=true);
    PhaseSystem.Voltage v[PhaseSystem.n] "voltage vector";
    flow PhaseSystem.Current i[PhaseSystem.n] "current vector";
    PhaseSystem.ReferenceAngle theta[PhaseSystem.m] "vector of phase angles";
  end Terminal;

  connector Terminal_p "Positive terminal"
    extends PowerFlow.Interfaces.Terminal(v(each start=1e3, each nominal=1e3), i(each start
          =                                                                                 1e3, each
          nominal =                                                                                           1e3));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}), graphics={Polygon(
            points={{100,100},{-100,100},{-100,-100},{100,-100},{100,100}},
            lineColor={0,0,0},
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid), Text(
            extent={{-150,-90},{150,-150}},
            lineColor={0,0,0},
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid,
            textString="%name")}),     Icon(coordinateSystem(
            preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
          graphics={Polygon(
            points={{100,100},{-100,100},{-100,-100},{100,-100},{100,100}},
            lineColor={0,0,0},
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid)}));
  end Terminal_p;

  connector Terminal_n "Negative terminal"
    extends PowerFlow.Interfaces.Terminal(v(each start=1e3, each nominal=1e3), i(each start
          =                                                                                 -1e3, each
          nominal =                                                                                            1e3));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}), graphics={
          Polygon(
            points={{100,100},{-100,100},{-100,-100},{100,-100},{100,100}},
            lineColor={0,0,0},
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid),
          Text(
            extent={{-150,-90},{150,-150}},
            lineColor={0,0,0},
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid,
            textString="%name"),
          Polygon(
            points={{70,70},{-70,70},{-70,-70},{70,-70},{70,70}},
            lineColor={255,255,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}),         Icon(coordinateSystem(
            preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
          graphics={Polygon(
            points={{100,100},{-100,100},{-100,-100},{100,-100},{100,100}},
            lineColor={0,0,0},
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid), Polygon(
            points={{70,70},{-70,70},{-70,-70},{70,-70},{70,70}},
            lineColor={255,255,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}));
  end Terminal_n;

  partial model PartialTwoTerminal
    replaceable package PhaseSystem = PackagePhaseSystem constrainedby
      PowerFlow.PhaseSystems.PartialPhaseSystem "Phase system" 
      annotation (choicesAllMatching=true);
    function j = PhaseSystem.j;
    PowerFlow.Interfaces.Terminal_p terminal_p(
                                      redeclare package PhaseSystem = 
          PhaseSystem)                                           annotation (Placement(
          transformation(extent={{-110,-10},{-90,10}}, rotation=0)));
    PowerFlow.Interfaces.Terminal_n terminal_n(
                                      redeclare package PhaseSystem = 
          PhaseSystem)                                           annotation (Placement(
          transformation(extent={{90,-10},{110,10}}, rotation=0)));
    PhaseSystem.Voltage v[:] = terminal_p.v - terminal_n.v;
    PhaseSystem.Current i[:] = terminal_p.i;
    PhaseSystem.Power S[PhaseSystem.n] = PhaseSystem.systemPower(PhaseSystem.phasePowers_vi(v, i));
    PhaseSystem.Voltage V = PhaseSystem.systemVoltage(v);
    PhaseSystem.Current I = PhaseSystem.systemCurrent(i);
    PhaseSystem.PhaseAngle phi = PhaseSystem.phase(v) - PhaseSystem.phase(i);
  equation
    Connections.branch(terminal_p.theta, terminal_n.theta);
  end PartialTwoTerminal;

  partial model PartialSource
    replaceable package PhaseSystem = PackagePhaseSystem constrainedby
      PowerFlow.PhaseSystems.PartialPhaseSystem "Phase system" 
      annotation (choicesAllMatching=true);
    function j = PhaseSystem.j;
    PowerFlow.Interfaces.Terminal_n terminal(
                                    redeclare package PhaseSystem = 
          PhaseSystem) 
      annotation (Placement(transformation(extent={{90,-10},{110,10}},
            rotation=0)));
    PhaseSystem.Power S[PhaseSystem.n] = -PhaseSystem.systemPower(PhaseSystem.phasePowers_vi(terminal.v, terminal.i));
    PhaseSystem.PhaseAngle phi = PhaseSystem.phase(terminal.v) - PhaseSystem.phase(-terminal.i);
    parameter Boolean potentialReference = true "serve as potential root" 
       annotation (Evaluate=true, Dialog(group="Reference Parameters"));
    parameter Boolean definiteReference = false "serve as definite root" 
       annotation (Evaluate=true, Dialog(group="Reference Parameters"));
  equation
    if potentialReference then
      if definiteReference then
        Connections.root(terminal.theta);
      else
        Connections.potentialRoot(terminal.theta);
      end if;
    end if;
  end PartialSource;

  partial model PartialLoad
    replaceable package PhaseSystem = PackagePhaseSystem constrainedby
      PowerFlow.PhaseSystems.PartialPhaseSystem "Phase system" 
      annotation (choicesAllMatching=true);
    function j = PhaseSystem.j;
    PowerFlow.Interfaces.Terminal_p terminal(
                                    redeclare package PhaseSystem = 
          PhaseSystem) 
      annotation (Placement(transformation(extent={{-110,-10},{-90,10}},
            rotation=0)));
    PhaseSystem.Voltage v[:] = terminal.v;
    PhaseSystem.Current i[:] = terminal.i;
    PhaseSystem.Power S[PhaseSystem.n] = PhaseSystem.systemPower(PhaseSystem.phasePowers_vi(v, i));
  end PartialLoad;
end Interfaces;
