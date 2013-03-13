within ;
package InstantaneousSymmetricalComponents "Sensors"

  package Examples
    extends Modelica.Icons.ExamplesPackage;

    model InstantaneousSymmetricalComponentsSensors
      extends Modelica.Icons.Example;
      parameter Integer m = 3 "Number of phases";
      Modelica.Electrical.MultiPhase.Sources.SineVoltage sineVoltage3(
        m=3,
        V=fill(1, 3),
        freqHz=fill(1, 3))
        annotation (Placement(transformation(
            extent={{10,10},{-10,-10}},
            rotation=90,
            origin={-50,80})));
      Modelica.Electrical.MultiPhase.Basic.Star star3(m=3)
        annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-50,50})));
      Modelica.Electrical.Analog.Basic.Ground ground3
        annotation (Placement(transformation(extent={{-60,8},{-40,28}})));
      Modelica.Electrical.MultiPhase.Sensors.VoltageSensor voltageSensor3(m=3)
        annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=90,
            origin={-30,80})));
      Modelica.Electrical.Machines.SpacePhasors.Blocks.ToSpacePhasor toSpacePhasor
        annotation (Placement(transformation(extent={{0,70},{20,90}})));
      Modelica.Electrical.Machines.SpacePhasors.Blocks.FromSpacePhasor
        fromSpacePhasor
        annotation (Placement(transformation(extent={{40,70},{60,90}})));
      Modelica.Electrical.MultiPhase.Sources.SineVoltage sineVoltage(
        m=m,
        V=fill(1, m),
        freqHz=fill(1, m))
       annotation (Placement(transformation(
            extent={{10,10},{-10,-10}},
            rotation=90,
            origin={-50,-20})));
      Modelica.Electrical.MultiPhase.Basic.Star star(m=m)
        annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-50,-50})));
      Modelica.Electrical.Analog.Basic.Ground ground
        annotation (Placement(transformation(extent={{-60,-92},{-40,-72}})));
      Modelica.Electrical.MultiPhase.Sensors.VoltageSensor voltageSensor(m=m)
        annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=90,
            origin={-30,-20})));

      Blocks.InstantaneousSymmetricalComponents instantaneousSymmetricalComponents(m=m)
        annotation (Placement(transformation(extent={{0,-30},{20,-10}})));
      Blocks.InverseInstantaneousSymmetricalComponents inverseInstantaneousSymmetricalComponents(m=m)
        annotation (Placement(transformation(extent={{40,-30},{60,-10}})));
    equation
      connect(sineVoltage3.plug_n, star3.plug_p)
                                               annotation (Line(
          points={{-50,70},{-50,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(star3.pin_n, ground3.p)
                                    annotation (Line(
          points={{-50,40},{-50,28}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(sineVoltage3.plug_p, voltageSensor3.plug_p) annotation (Line(
          points={{-50,90},{-30,90}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(sineVoltage3.plug_n, voltageSensor3.plug_n) annotation (Line(
          points={{-50,70},{-30,70}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(voltageSensor3.v, toSpacePhasor.u) annotation (Line(
          points={{-19,80},{-2,80}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(toSpacePhasor.y, fromSpacePhasor.u) annotation (Line(
          points={{21,80},{38,80}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(toSpacePhasor.zero, fromSpacePhasor.zero) annotation (Line(
          points={{21,72},{38,72}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(sineVoltage.plug_n,star. plug_p) annotation (Line(
          points={{-50,-30},{-50,-40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(star.pin_n,ground. p) annotation (Line(
          points={{-50,-60},{-50,-72}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(sineVoltage.plug_p, voltageSensor.plug_p) annotation (Line(
          points={{-50,-10},{-30,-10}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(sineVoltage.plug_n, voltageSensor.plug_n) annotation (Line(
          points={{-50,-30},{-30,-30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(voltageSensor.v, instantaneousSymmetricalComponents.u) annotation (
          Line(
          points={{-19,-20},{-2,-20}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(instantaneousSymmetricalComponents.y,
        inverseInstantaneousSymmetricalComponents.u) annotation (Line(
          points={{21,-20},{38,-20}},
          color={85,170,255},
          smooth=Smooth.None));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics));
    end InstantaneousSymmetricalComponentsSensors;
  end Examples;

  package Sensors

    model VoltageInstantaneousSymmetricalComponentsSensor
      "Continuous instantaneous symmetrical components voltage sensor for multi phase system"
      extends Modelica.Icons.RotationalSensor;
      extends Modelica.Electrical.MultiPhase.Interfaces.TwoPlug;
      Modelica.ComplexBlocks.Interfaces.ComplexOutput V[m]
        "Instantaneous symmetrical voltage components"
        annotation (Placement(transformation(
            origin={0,-100},
            extent={{-10,-10},{10,10}},
            rotation=270)));
      Modelica.Electrical.MultiPhase.Sensors.VoltageSensor currentSensor(final m=m)
        annotation (Placement(transformation(extent={{-10,-10},{10,10}},
              rotation=0)));
      Blocks.InstantaneousSymmetricalComponents instantaneousSymmetricalComponents(m=m)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,-30})));
    equation
      connect(plug_p,currentSensor. plug_p)
        annotation (Line(points={{-100,5.55112e-016},{-100,0},{-10,0}},
                                                               color={0,0,255}));
      connect(currentSensor.plug_n, plug_n)
        annotation (Line(points={{10,0},{100,0},{100,5.55112e-016}},
                                                            color={0,0,255}));
      connect(instantaneousSymmetricalComponents.y, V) annotation (Line(
          points={{0,-41},{0,-100}},
          color={85,170,255},
          smooth=Smooth.None));
      connect(currentSensor.v, instantaneousSymmetricalComponents.u) annotation (
          Line(
          points={{0,-11},{0,-18}},
          color={0,0,127},
          smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}),
                            graphics),
        Icon(graphics={Line(points={{0,-70},{0,-90}}, color={0,0,255}), Line(
                points={{-90,0},{-70,0}},color={0,0,255}),
            Text(
              extent={{-100,60},{100,120}},
              textString="%name",
              lineColor={0,0,255}),
            Text(
              extent={{-100,-60},{-20,-100}},
              lineColor={0,0,0},
              textString="m="),
            Text(
              extent={{20,-60},{100,-100}},
              lineColor={0,0,0},
              textString="%m"),                                         Line(
                points={{70,0},{90,0}},  color={0,0,255})}),
        Documentation(revisions="<html>
</html>",             info="<html>
<p>
This sensor determines the continuous quasi <a href=\"Modelica://Modelica.Blocks.Math.RootMeanSquare\">RMS</a> value of a multi phase voltage system, representiong an equivalent RMS current vector <code>I</code> or phasor. If the current waveform deviates from a sine curve, the output of the sensor will not be exactly the average RMS value.
</p>
<pre>
 V = sqrt(sum(v[k]^2 for k in 1:m)/m) 
</pre>
</html>"));
    end VoltageInstantaneousSymmetricalComponentsSensor;

    model CurrentInstantaneousSymmetricalComponentsSensor
      "Continuous instantaneous symmetrical components current sensor for multi phase system"
      extends Modelica.Icons.RotationalSensor;
      extends Modelica.Electrical.MultiPhase.Interfaces.TwoPlug;
      Modelica.ComplexBlocks.Interfaces.ComplexOutput I[m]
        "Instantaneous symmetrical voltage components"
        annotation (Placement(transformation(
            origin={0,-100},
            extent={{-10,-10},{10,10}},
            rotation=270)));
      Modelica.Electrical.MultiPhase.Sensors.CurrentSensor currentSensor(final m=m)
        annotation (Placement(transformation(extent={{-10,-10},{10,10}},
              rotation=0)));
      Blocks.InstantaneousSymmetricalComponents instantaneousSymmetricalComponents(m=m)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,-30})));
    equation
      connect(plug_p,currentSensor. plug_p)
        annotation (Line(points={{-100,5.55112e-016},{-100,0},{-10,0}},
                                                               color={0,0,255}));
      connect(currentSensor.plug_n, plug_n)
        annotation (Line(points={{10,0},{100,0},{100,5.55112e-016}},
                                                            color={0,0,255}));
      connect(currentSensor.i, instantaneousSymmetricalComponents.u) annotation (
          Line(
          points={{0,-11},{0,-18}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(instantaneousSymmetricalComponents.y, I) annotation (Line(
          points={{0,-41},{0,-100}},
          color={85,170,255},
          smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}),
                            graphics),
        Icon(graphics={Line(points={{0,-70},{0,-90}}, color={0,0,255}),
            Text(
              extent={{-100,60},{100,120}},
              textString="%name",
              lineColor={0,0,255}),
            Text(
              extent={{-100,-60},{-20,-100}},
              lineColor={0,0,0},
              textString="m="),
            Text(
              extent={{20,-60},{100,-100}},
              lineColor={0,0,0},
              textString="%m"),                                         Line(
                points={{70,0},{90,0}},  color={0,0,255}),              Line(
                points={{-90,0},{-70,0}},color={0,0,255})}),
        Documentation(revisions="<html>
</html>",             info="<html>
<p>
This sensor determines the continuous quasi <a href=\"Modelica://Modelica.Blocks.Math.RootMeanSquare\">RMS</a> value of a multi phase current system, representiong an equivalent RMS current vector <code>I</code> or phasor. If the current waveform deviates from a sine curve, the output of the sensor will not be exactly the average RMS value.
</p>
<pre>
 I = sqrt(sum(i[k]^2 for k in 1:m)/m) 
</pre>

</html>"));
    end CurrentInstantaneousSymmetricalComponentsSensor;

  end Sensors;

  package Blocks "Multi phase specific bocks operating on multi phase signals"
    extends Modelica.Icons.Package;
    block InstantaneousSymmetricalComponents
      "Determines the symmetrical components of instantaneous values"
      extends Modelica.Blocks.Icons.Block;
      parameter Integer m(min=1) = 3 "Number of phases";
      Modelica.Blocks.Interfaces.RealInput u[m]
        "Connector of Real input signals"                                         annotation (Placement(
            transformation(extent={{-140,-20},{-100,20}}, rotation=0)));
      Modelica.ComplexBlocks.Interfaces.ComplexOutput y[m]
        "Connector of Complex output signals (instantaneous symmetrical components)"
        annotation (Placement(transformation(extent={{100,-10},{120,10}}, rotation=0)));
    protected
      final parameter Complex a = Modelica.ComplexMath.exp(Complex(0, 2*Modelica.Constants.pi/m));
    equation
    //  y = Functions.instantaneuousSymmetricalComponents(u);
      y =  {{a^((k-1)*(l-1)) for k in 1:m} for l in 1:m}/sqrt(m)*u;
    end InstantaneousSymmetricalComponents;

    block InverseInstantaneousSymmetricalComponents
      "Determines the symmetrical components of instantaneous values"
      extends Modelica.Blocks.Interfaces.MO(final nout=m);
      parameter Integer m(min=1) = 3 "Number of phases";
      Modelica.ComplexBlocks.Interfaces.ComplexInput u[m]
        "Connector of Complex input signals"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}}, rotation=0)));
    protected
      final parameter Complex a = Modelica.ComplexMath.exp(Complex(0, 2*Modelica.Constants.pi/m));
      Complex yCalc "Calculated complex y from which real part is then taken";
    //equation
    //  y = Functions.inverseInstantaneuousSymmetricalComponents(u);
    algorithm
      for k in 1:m loop
        yCalc := Modelica.ComplexMath.conj({a^((k-1)*(l-1)) for l in 1:m})/sqrt(m)*u;
        assert(yCalc.im < 1000*Modelica.Constants.eps, "Input vector in inverseInstantaneuousSymmetricalComponents is not consistent");
        y[k] := yCalc.re;
      end for;
    end InverseInstantaneousSymmetricalComponents;
  end Blocks;

  package Functions "Multi phase specific functions"
    extends Modelica.Icons.Package;

    function instantaneuousSymmetricalComponents
      input Real u[:] "Real multi phase input";
      output Complex y[size(u,1)]
        "Symmetrical components of instantaneous components of input";
    protected
      final parameter Integer m = size(u,1) "Number of phases";
      final parameter Complex a = Modelica.ComplexMath.exp(Complex(0, 2*Modelica.Constants.pi/m));
    algorithm
      y := {{a^((k-1)*(l-1)) for k in 1:m} for l in 1:m}/sqrt(m)*u;
    annotation(inverse(u=inverseInstantaneuousSymmetricalComponents(y)));
    end instantaneuousSymmetricalComponents;

    function inverseInstantaneuousSymmetricalComponents
      input Complex u[:] "Symmetrical components";
      output Real y[size(u,1)] "Real multi phase output";
    protected
      final parameter Integer m = size(u,1) "Number of phases";
      final parameter Complex a = Modelica.ComplexMath.exp(Complex(0, 2*Modelica.Constants.pi/m));
      Complex yCalc "Calculated complex y from which real part is then taken";
    algorithm
      for k in 1:m loop
        yCalc := Modelica.ComplexMath.conj({a^((k-1)*(l-1)) for l in 1:m})/sqrt(m)*u;
        assert(yCalc.im < 1000*Modelica.Constants.eps, "Input vector in inverseInstantaneuousSymmetricalComponents is not consistent");
        y[k] := yCalc.re;
      end for;
    annotation(inverse(u=instantaneuousSymmetricalComponents(y)));
    end inverseInstantaneuousSymmetricalComponents;
  end Functions;

  annotation (uses(Modelica(version="3.2.1"), Complex(version="3.2.1")));
end InstantaneousSymmetricalComponents;
