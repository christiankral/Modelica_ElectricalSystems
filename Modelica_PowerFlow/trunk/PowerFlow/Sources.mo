within PowerFlow;
package Sources
  model FixedVoltageSource
    extends Interfaces.PartialSource;
    parameter PhaseSystem.Voltage V = 10e3 "value of constant voltage";
    parameter PhaseSystem.Frequency w_ref = 50
      "frequency of sinusoidal voltage" 
      annotation (Dialog(group="Reference Parameters"));
  equation
    terminal.v = PhaseSystem.phaseVoltages(V, 0);
    if isRoot(terminal.theta) and PhaseSystem.m > 0 then
      PhaseSystem.angle(terminal.theta) =  w_ref*time;
    end if;
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}), graphics={Rectangle(
            extent={{-90,90},{90,-90}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.CrossDiag)}),
                               Diagram(coordinateSystem(preserveAspectRatio=false,
                     extent={{-100,-100},{100,100}}),
                                       graphics));
  end FixedVoltageSource;

  model FixedLoad
    extends Interfaces.PartialLoad;
    parameter Modelica.SIunits.Power P = 0 "rms value of constant active power";
    parameter Modelica.SIunits.Angle phi = 0 "phase angle";
  equation
    PhaseSystem.phasePowers_vi(terminal.v, terminal.i) = PhaseSystem.phasePowers(P, phi);
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}),
                        graphics),
                         Icon(coordinateSystem(preserveAspectRatio=false,
            extent={{-100,-100},{100,100}}), graphics={Rectangle(
            extent={{-90,90},{90,-90}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid), Text(
            extent={{-38,-68},{38,-26}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="VA")}));
  end FixedLoad;

  model FixedCurrent
    extends Interfaces.PartialLoad;
    parameter Modelica.SIunits.Current I = 0 "rms value of constant current";
    parameter Modelica.SIunits.Angle phi = 0 "phase angle" 
    annotation (Dialog(group="Reference Parameters", enable = definiteReference));
  equation
    terminal.i = PhaseSystem.phaseCurrents(I, phi);
    annotation (Diagram(graphics),
                        Icon(coordinateSystem(preserveAspectRatio=false,
            extent={{-100,-100},{100,100}}), graphics={
          Rectangle(
            extent={{-90,90},{90,-90}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Text(
            extent={{-78,22},{80,64}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="%I% A"),
          Text(
            extent={{-38,-74},{38,-32}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="%phi%")}));
  end FixedCurrent;

  model PrescribedPowerSource "Prescribed power source"
    extends Interfaces.PartialSource(
      final potentialReference=true);
    parameter PhaseSystem.Frequency w_ref = 50
      "frequency of sinusoidal voltage" 
      annotation (Dialog(group="Reference Parameters"));
    Modelica.Blocks.Interfaces.RealInput P(unit="MW") annotation (Placement(
          transformation(extent={{-130,-20},{-90,20}}, rotation=0)));
    PhaseSystem.Current I "value of current";
  equation
    terminal.i = PhaseSystem.phaseCurrents(I, 0);
    0 = PhaseSystem.systemPower(terminal.v*terminal.i) + 1e6*P;
    if isRoot(terminal.theta) and PhaseSystem.m > 0 then
      PhaseSystem.angle(terminal.theta) =  w_ref*time;
    end if;
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}),
                        graphics),
                         Icon(coordinateSystem(preserveAspectRatio=false,
            extent={{-100,-100},{100,100}}), graphics={Rectangle(
            extent={{-90,90},{90,-90}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid), Text(
            extent={{-90,-132},{90,-90}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="%name%")}));
  end PrescribedPowerSource;

  model PrescribedPowerLoad "Prescribed power load"
    extends Interfaces.PartialLoad;
    parameter Modelica.SIunits.Angle phi = 0 "phase angle";
    Modelica.Blocks.Interfaces.RealInput P(unit="MW") annotation (Placement(
          transformation(extent={{130,-20},{90,20}}, rotation=0)));
  equation
    PhaseSystem.phasePowers_vi(terminal.v, terminal.i) = PhaseSystem.phasePowers(1e6*P, phi);
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}),
                        graphics),
                         Icon(coordinateSystem(preserveAspectRatio=false,
            extent={{-100,-100},{100,100}}), graphics={Rectangle(
            extent={{-90,90},{90,-90}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid), Text(
            extent={{-90,-132},{90,-90}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="%name%")}));
  end PrescribedPowerLoad;
end Sources;
