within Modelica_FundamentalWave;
package MoveToModelica "To be moved to MSL 3.X"
  model SwitchedRheostat "Rheostat which is shortened after a given time"

    constant Integer m= 3 "Number of phases";
    Modelica.Electrical.MultiPhase.Interfaces.PositivePlug plug_p(
      final m=m) 
      annotation (Placement(transformation(extent={{90,70},{110,50}}, rotation=
              0)));
    Modelica.Electrical.MultiPhase.Interfaces.NegativePlug plug_n(
      final m=m) 
      annotation (Placement(transformation(extent={{90,-50},{110,-70}},
            rotation=0)));

    parameter Modelica.SIunits.Resistance RStart "Starting resistance";
    parameter Modelica.SIunits.Time tStart
      "Duration of switching on the starting resistor";

    Modelica.Electrical.MultiPhase.Basic.Star star(
      final m=m) 
      annotation (Placement(transformation(extent={{-40,-70},{-60,-50}},
            rotation=0)));
    Modelica.Electrical.Analog.Basic.Ground ground 
      annotation (Placement(transformation(
          origin={-80,-60},
          extent={{-10,-10},{10,10}},
          rotation=270)));
    Modelica.Electrical.MultiPhase.Ideal.IdealCommutingSwitch
      idealCommutingSwitch(
      final m=m) 
      annotation (Placement(transformation(
          origin={40,20},
          extent={{-10,10},{10,-10}},
          rotation=270)));
    Modelica.Electrical.MultiPhase.Basic.Resistor rheostat(
      final m=m,
      final R=fill(RStart, m)) 
      annotation (Placement(transformation(extent={{0,-30},{-20,-10}}, rotation
            =0)));
    Modelica.Electrical.MultiPhase.Basic.Star starRheostat(
      final m=m) 
      annotation (Placement(transformation(extent={{-40,-30},{-60,-10}},
            rotation=0)));
    Modelica.Blocks.Sources.BooleanStep booleanStep[m](
      final startTime=fill(tStart, m),
      final startValue=fill(false, m)) 
      annotation (Placement(transformation(extent={{-60,10},{-40,30}}, rotation
            =0)));

  equation
    connect(star.pin_n, ground.p) 
      annotation (Line(points={{-60,-60},{-70,-60}}, color={0,0,255}));
    connect(starRheostat.plug_p, rheostat.plug_n) 
      annotation (Line(points={{-40,-20},{-20,-20}}, color={0,0,255}));
    connect(rheostat.plug_p, idealCommutingSwitch.plug_n1) 
      annotation (Line(points={{5.55112e-16,-20},{35,-20},{35,10}}, color={0,0,
            255}));
    connect(booleanStep.y, idealCommutingSwitch.control)   annotation (Line(
          points={{-39,20},{32,20}}, color={255,0,255}));
    annotation (Diagram(graphics),
                         Icon(graphics={
          Rectangle(
            extent={{26,40},{54,-40}},
            lineColor={0,0,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Line(points={{100,60},{-40,60},{-40,40}}, color={0,0,255}),
          Line(points={{100,-60},{-40,-60},{-40,-40}}, color={0,0,255}),
          Ellipse(extent={{-44,40},{-36,32}}, lineColor={0,0,255}),
          Ellipse(extent={{-44,-32},{-36,-40}}, lineColor={0,0,255}),
          Line(points={{-80,40},{-42,-34}}, color={0,0,255}),
          Line(points={{40,40},{40,42},{40,60}}, color={0,0,255}),
          Line(points={{40,-40},{40,-60}}, color={0,0,255}),
          Line(points={{10,-80},{70,-80}}, color={0,0,255}),
          Line(points={{40,-60},{40,-80}}, color={0,0,255}),
          Line(points={{20,-90},{60,-90}}, color={0,0,255}),
          Line(points={{30,-100},{50,-100}}, color={0,0,255})}));
    connect(idealCommutingSwitch.plug_p, plug_p)  annotation (Line(points={{40,
            30},{40,60},{100,60}}, color={0,0,255}));

    connect(idealCommutingSwitch.plug_n2, plug_n)  annotation (Line(points={{40,
            10},{40,-60},{100,-60}}, color={0,0,255}));
    connect(star.plug_p, idealCommutingSwitch.plug_n2)   annotation (Line(
          points={{-40,-60},{40,-60},{40,10}}, color={0,0,255}));
    connect(starRheostat.pin_n, star.pin_n) annotation (Line(points={{-60,-20},
            {-60,-60}}, color={0,0,255}));
  end SwitchedRheostat;

  model RampedRheostat "Rheostat with linearly decreasing resistance"

    constant Integer m= 3 "Number of phases";
    Modelica.Electrical.MultiPhase.Interfaces.PositivePlug plug_p(
      final m=m) 
      annotation (Placement(transformation(extent={{90,70},{110,50}}, rotation=
              0)));
    Modelica.Electrical.MultiPhase.Interfaces.NegativePlug plug_n(
      final m=m) 
      annotation (Placement(transformation(extent={{90,-50},{110,-70}},
            rotation=0)));

    parameter Modelica.SIunits.Resistance RStart "Starting resistance";
    parameter Modelica.SIunits.Time tStart
      "Time instance of reducing the rheostat";
    parameter Modelica.SIunits.Time tRamp "Duration of ramp";

    Modelica.Electrical.MultiPhase.Basic.Star star(
      final m=m) 
      annotation (Placement(transformation(extent={{-20,-70},{-40,-50}},
            rotation=0)));
    Modelica.Electrical.Analog.Basic.Ground ground 
      annotation (Placement(transformation(
          origin={-70,-60},
          extent={{-10,-10},{10,10}},
          rotation=270)));
    Modelica.Electrical.MultiPhase.Basic.VariableResistor rheostat(
      final m=m) 
      annotation (Placement(transformation(
          origin={60,0},
          extent={{-10,10},{10,-10}},
          rotation=270)));
    Modelica.Blocks.Sources.Ramp ramp[m](
      final height=fill(-RStart, m),
      final duration=fill(tRamp, m),
      final offset=fill(RStart, m),
      final startTime=fill(tStart, m)) 
      annotation (Placement(transformation(extent={{-20,-10},{0,10}}, rotation=
              0)));
  equation
    connect(star.pin_n, ground.p) 
      annotation (Line(points={{-40,-60},{-60,-60}}, color={0,0,255}));
    connect(rheostat.plug_p, plug_p) annotation (Line(points={{60,10},{60,60},{
            100,60}}, color={0,0,255}));

    connect(rheostat.plug_n, plug_n) annotation (Line(points={{60,-10},{60,-60},
            {100,-60}}, color={0,0,255}));
    connect(star.plug_p, plug_n) annotation (Line(points={{-20,-60},{100,-60}}, 
          color={0,0,255}));
    connect(ramp.y, rheostat.R) annotation (Line(points={{1,6.10623e-16},{24.5,
            6.10623e-16},{24.5,-1.33731e-15},{50,-1.33731e-15}}, color={0,0,127}));

    annotation (Diagram(graphics),
                         Icon(graphics={
          Rectangle(
            extent={{26,40},{54,-40}},
            lineColor={0,0,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Line(points={{100,-60},{40,-60},{40,-40}}, color={0,0,255}),
          Line(points={{10,-80},{70,-80}}, color={0,0,255}),
          Line(points={{40,-60},{40,-80}}, color={0,0,255}),
          Line(points={{20,-90},{60,-90}}, color={0,0,255}),
          Line(points={{30,-100},{50,-100}}, color={0,0,255}),
          Line(points={{40,40},{40,60},{100,60}}, color={0,0,255}),
          Rectangle(
            extent={{-100,40},{-20,-40}},
            lineColor={0,0,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Line(points={{-90,24},{-72,24},{-30,-30}}, color={0,0,255}),
          Polygon(
            points={{-20,6},{-10,0},{-20,-6},{-20,6}},
            lineColor={0,0,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{10,6},{20,0},{10,-6},{10,6}},
            lineColor={0,0,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Line(points={{-10,0},{10,0}}, color={0,0,255}),
          Line(points={{20,0},{26,0}}, color={0,0,255})}));

  end RampedRheostat;
end MoveToModelica;
