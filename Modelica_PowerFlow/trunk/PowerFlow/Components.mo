within PowerFlow;
package Components
  model Impedance
    extends PowerFlow.Interfaces.PartialTwoTerminal;
    parameter Modelica.SIunits.Resistance R = 1 "active component";
    parameter Modelica.SIunits.Inductance L = 1/50 "reactive component";
    PhaseSystem.Frequency w = der(PhaseSystem.angle(terminal_p.theta));
  equation
    v = R*i + w*L*j(i);
    zeros(PhaseSystem.n) = terminal_p.i + terminal_n.i;
    terminal_p.theta = terminal_n.theta;
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}),
                        graphics),
                         Icon(coordinateSystem(preserveAspectRatio=false,
            extent={{-100,-100},{100,100}}), graphics={
          Rectangle(
            extent={{-70,30},{70,-30}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Line(points={{-100,0},{-70,0}}, color={0,0,0}),
          Line(points={{70,0},{100,0}}, color={0,0,0}),
          Text(
            extent={{-144,-60},{144,-100}},
            lineColor={0,0,0},
            textString="R=%R%, L=%L%"),
          Text(
            extent={{-144,40},{144,100}},
            lineColor={0,0,0},
            textString="%name"),
          Rectangle(
            extent={{0,10},{66,-10}},
            lineColor={0,0,0},
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid)}));
  end Impedance;

  model Admittance
    extends Interfaces.PartialTwoTerminal;
    parameter Modelica.SIunits.Conductance G = 1 "active component";
    parameter Modelica.SIunits.Capacitance C = 1/50 "reactive component";
    PhaseSystem.Frequency w = der(PhaseSystem.angle(terminal_p.theta));
  equation
    i = G*v + w*C*j(v);
    zeros(PhaseSystem.n) = terminal_p.i + terminal_n.i;
    terminal_p.theta = terminal_n.theta;
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{
              -100,-100},{100,100}}),
                        graphics),
                         Icon(coordinateSystem(preserveAspectRatio=false,
            extent={{-100,-100},{100,100}}), graphics={
          Rectangle(
            extent={{-70,30},{70,-30}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Line(points={{-100,0},{-70,0}}, color={0,0,0}),
          Line(points={{70,0},{100,0}}, color={0,0,0}),
          Text(
            extent={{-144,-60},{144,-100}},
            lineColor={0,0,0},
            textString="G=%G%, C=%C%"),
          Text(
            extent={{-144,40},{144,100}},
            lineColor={0,0,0},
            textString="%name"),
          Rectangle(
            extent={{14,30},{24,-30}},
            lineColor={0,0,0},
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{36,30},{46,-30}},
            lineColor={0,0,0},
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid)}));
  end Admittance;

  model VoltageConverter
    extends PowerFlow.Interfaces.PartialTwoTerminal;
    parameter Real ratio = 1 "conversion ratio terminal_p.v/terminal_n.v";
  equation
    terminal_p.v = ratio*terminal_n.v;
    zeros(PhaseSystem.n) = ratio*terminal_p.i + terminal_n.i;
    terminal_p.theta = terminal_n.theta;
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}),
                        graphics),
                         Icon(coordinateSystem(preserveAspectRatio=false,
            extent={{-100,-100},{100,100}}), graphics={
          Line(points={{-100,0},{-70,0}}, color={0,0,0}),
          Line(points={{70,0},{100,0}}, color={0,0,0}),
          Text(
            extent={{-144,-60},{144,-100}},
            lineColor={0,0,0},
            textString="%ratio%"),
          Text(
            extent={{-144,60},{144,120}},
            lineColor={0,0,0},
            textString="%name"),
          Ellipse(
            extent={{-80,50},{20,-50}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{-20,50},{80,-50}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Ellipse(extent={{-80,50},{20,-50}}, lineColor={0,0,0})}));
  end VoltageConverter;

  model Ground
    extends Interfaces.PartialLoad;
  equation
    terminal.v = zeros(PhaseSystem.n);
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}), graphics={
          Line(points={{0,0},{0,-60}}, color={0,0,0}),
          Line(points={{-80,-60},{80,-60}}, color={0,0,0}),
          Line(points={{-50,-80},{50,-80}}, color={0,0,0}),
          Line(points={{-20,-100},{20,-100}}, color={0,0,0}),
          Line(points={{-100,0},{0,0}}, color={0,0,0})}),
                               Diagram(graphics));
  end Ground;

  model EMF "Electro-Motoric Force"
    extends Interfaces.PartialSource(final potentialReference = synchronous);
    parameter Boolean synchronous = PhaseSystem.m > 0 "synchronous machine";
    parameter PhaseSystem.Frequency w_ref = 50 "reference value of frequency"
      annotation (Dialog(group="Reference Parameters"));
    parameter PhaseSystem.Voltage V_ref = 10e3 "reference value of voltage"
      annotation (Dialog(group="Reference Parameters"));
    Modelica.Mechanics.Rotational.Interfaces.Flange_a flange
      annotation (Placement(transformation(extent={{-110,-10},{-90,10}},
            rotation=0)));
    PhaseSystem.Frequency w = der(flange.phi);
    PhaseSystem.Voltage V;
  equation
    0 = PhaseSystem.systemPower(terminal.v*terminal.i) + w*flange.tau;
    terminal.v = PhaseSystem.phaseVoltages(V, 0);
    if synchronous then
      flange.phi = PhaseSystem.angle(terminal.theta);
      if isRoot(terminal.theta) then
        V = V_ref;
      end if;
    else
      V = V_ref/w_ref*w;
    end if;
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}),
                        graphics),
                         Icon(coordinateSystem(preserveAspectRatio=true,  extent={{-100,
              -100},{100,100}}), graphics={
          Line(points={{-100,0},{-50,0}}, color={0,0,0}),
          Line(points={{50,0},{100,0}}, color={0,0,0}),
          Text(
            extent={{-144,-60},{144,-100}},
            lineColor={0,0,0},
            textString="V=%V_ref V"),
          Text(
            extent={{-144,40},{144,100}},
            lineColor={0,0,0},
            textString="%name"),
          Ellipse(
            extent={{-50,50},{50,-50}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Text(
            extent={{-40,30},{40,-30}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="A",
            visible=not synchronous),
          Rectangle(
            extent={{-28,30},{30,-30}},
            lineColor={255,255,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            visible=synchronous),
          Text(
            extent={{-40,30},{40,-30}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="S",
            visible=synchronous)}));
  end EMF;

  model Inverter "Convert direct current to alternating current"
    extends Interfaces.PartialSource(final potentialReference = true);
    package PhaseSystem_dc = PowerFlow.PhaseSystems.DirectCurrent;
    PowerFlow.Interfaces.Terminal_p terminal_dc(
      redeclare package PhaseSystem = PhaseSystem_dc)
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}}, rotation=0)));
    parameter PhaseSystem_dc.Voltage V_dc = 150e3 "voltage of dc system";
    parameter PhaseSystem.Frequency w_ref = 50
      "frequency of sinusoidal voltage"
      annotation (Dialog(group="Reference Parameters"));
    PhaseSystem.Current I "value of current";
  equation
    terminal_dc.v = PhaseSystem_dc.phaseVoltages(V_dc);
    terminal.i = PhaseSystem.phaseCurrents(I, 0);
    0 = PhaseSystem_dc.systemPower(terminal_dc.v*terminal_dc.i) + PhaseSystem.systemPower(terminal.v*terminal.i);
    if isRoot(terminal.theta) and PhaseSystem.m > 0 then
      PhaseSystem.angle(terminal.theta) = w_ref*time;
    end if;
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}),
                        graphics),
                         Icon(coordinateSystem(preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
          Rectangle(
            extent={{-90,90},{90,-90}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Text(
            extent={{0,-68},{80,12}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="~"),
          Line(
            points={{-90,-90},{90,90}},
            color={0,0,0},
            smooth=Smooth.None),
          Text(
            extent={{-68,-10},{12,70}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="="),
          Text(
            extent={{0,-84},{80,-4}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="~"),
          Text(
            extent={{0,-100},{80,-20}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="~"),
          Text(
            extent={{-142,98},{146,158}},
            lineColor={0,0,0},
            textString="%name")}));
  end Inverter;
end Components;
