within ;
package Modelica_Electrical_PowerConverters "Rectifiers and DC/DC converters"
  extends Modelica.Icons.Package;
  package Examples
    extends Modelica.Icons.ExamplesPackage;
    package FullBridge1Pulse "Single pulse rectifier"
      extends Modelica.Icons.ExamplesPackage;

      model FullBridge1Pulse_R
        extends
          Modelica_Electrical_PowerConverters.Examples.ExampleTemplates.FullBridge1Pulse(
            twoPulse(
            useSignal=false,
            f=f,
            alpha=alpha), idealthyristor(Vknee=0));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle alpha = 90 * pi / 180 "Firing angle";
        parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)    annotation(Placement(visible = true, transformation(origin={30,30},                     extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
      equation
        connect(idealthyristor.n, resistor.p) annotation (Line(
            points={{4.44089e-16,40},{30,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor.n, currentSensor.p) annotation (Line(
            points={{30,20},{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent={{-100,
                  -100},{100,100}},                                                                                                    preserveAspectRatio=false,  initialScale = 0.1, grid = {2,2}),
              graphics),                                                                                                    experiment(
            StopTime=0.1,
            __Dymola_NumberOfIntervals=5000,
            Tolerance=1e-06),
          __Dymola_experimentSetupOutput,
          Documentation(info="<html>
<p>Inductive load does not make sense, since average DC voltage is very low due to long conduction period of the thyristor. </p>
</html>"));
      end FullBridge1Pulse_R;

      model FullBridge1Pulse_R_Characteristic
        extends
          Modelica_Electrical_PowerConverters.Examples.ExampleTemplates.FullBridge1Pulse(
            twoPulse(useSignal=true, f=f), idealthyristor(Vknee=0));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)    annotation(Placement(visible = true, transformation(origin={30,30},                     extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Blocks.Sources.Ramp ramp(height=pi, duration=10)
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-40,70})));
        Modelica.Blocks.Sources.Ramp ramp1(
                                          height=pi, duration=10)
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-30,80})));
      equation
        connect(ramp.y, twoPulse.u) annotation (Line(
            points={{-40,59},{-40,10}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(resistor.n, currentSensor.p) annotation (Line(
            points={{30,20},{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor.p, idealthyristor.n) annotation (Line(
            points={{30,40},{0,40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent={{-100,
                  -100},{100,100}},                                                                                                    preserveAspectRatio=false,  initialScale = 0.1, grid = {2,2}),
              graphics),                                                                                                    experiment(
            StopTime=10,
            __Dymola_NumberOfIntervals=50000,
            Tolerance=1e-06),
          __Dymola_experimentSetupOutput,
          Documentation(info="<html>
<p>Inductive load does not make sense, since average DC voltage is very low due to long conduction period of the thyristor. </p>
</html>"));
      end FullBridge1Pulse_R_Characteristic;
    end FullBridge1Pulse;

    package FullBridge2PulseCenterTap "Examples of Power Electronics with M2C"
      extends Modelica.Icons.ExamplesPackage;
      model FullBridge2PulseCenterTap_R
        extends
          Modelica_Electrical_PowerConverters.Examples.ExampleTemplates.FullBridge2PulseCenterTap(
            pulse2m(alpha=alpha));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle alpha = 90 * pi / 180 "Firing angle";
        parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)    annotation(Placement(visible = true, transformation(origin={30,30},                     extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
      equation
        connect(resistor.n, currentSensor.p) annotation (Line(
            points={{30,20},{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent={{-100,
                  -100},{100,100}},                                                                                                    preserveAspectRatio=false,  initialScale = 0.1, grid = {2,2}),
              graphics),                                                                                                    experiment(
            StopTime=0.1,
            __Dymola_NumberOfIntervals=50000,
            Tolerance=1e-06),
          __Dymola_experimentSetupOutput);
      end FullBridge2PulseCenterTap_R;

      model FullBridge2PulseCenterTap_RL
        extends
          Modelica_Electrical_PowerConverters.Examples.ExampleTemplates.FullBridge2PulseCenterTap(
            pulse2m(alpha=alpha));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle alpha = 90 * pi / 180 "Firing angle";
        parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        parameter Modelica.SIunits.Inductance L = 1 "Load resistance" annotation(Evaluate=true);
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)    annotation(Placement(visible = true, transformation(origin={30,30},                     extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L)    annotation(Placement(visible = true, transformation(origin={30,0},         extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
      equation
        connect(resistor.n, inductor.p) annotation (Line(
            points={{30,20},{30,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(inductor.n, currentSensor.p) annotation (Line(
            points={{30,-10},{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent={{-100,
                  -100},{100,100}},                                                                                                    preserveAspectRatio=false,  initialScale = 0.1, grid = {2,2}),
              graphics),                                                                                                    experiment(
            StopTime=0.1,
            __Dymola_NumberOfIntervals=50000,
            Tolerance=1e-06),
          __Dymola_experimentSetupOutput);
      end FullBridge2PulseCenterTap_RL;

      model FullBridge2PulseCenterTap_RLV
        extends
          Modelica_Electrical_PowerConverters.Examples.ExampleTemplates.FullBridge2PulseCenterTap(
            pulse2m(alpha=alpha));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle alpha = 90 * pi / 180 "Firing angle";
        parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        parameter Modelica.SIunits.Inductance L = 1 "Load resistance" annotation(Evaluate=true);
        parameter Modelica.SIunits.Voltage VDC=-120 "DC load offset voltage";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)    annotation(Placement(visible = true, transformation(origin={30,30},                     extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L)    annotation(Placement(visible = true, transformation(origin={30,0},         extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=VDC)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={30,-30})));
      equation
        connect(resistor.n, inductor.p) annotation (Line(
            points={{30,20},{30,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(inductor.n, constantVoltage.p) annotation (Line(
            points={{30,-10},{30,-20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(constantVoltage.n, currentSensor.p) annotation (Line(
            points={{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent={{-100,
                  -100},{100,100}},                                                                                                    preserveAspectRatio=false,  initialScale = 0.1, grid = {2,2}),
              graphics),                                                                                                    experiment(
            StopTime=0.1,
            __Dymola_NumberOfIntervals=50000,
            Tolerance=1e-06),
          __Dymola_experimentSetupOutput);
      end FullBridge2PulseCenterTap_RLV;

      model FullBridge2PulseCenterTap_RLV_Characteristic
        extends
          Modelica_Electrical_PowerConverters.Examples.ExampleTemplates.FullBridge2PulseCenterTap;
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        parameter Modelica.SIunits.Inductance L = 1 "Load resistance" annotation(Evaluate=true);
        parameter Modelica.SIunits.Voltage VDC=-120 "DC load offset voltage";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)    annotation(Placement(visible = true, transformation(origin={30,30},                     extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L)    annotation(Placement(visible = true, transformation(origin={30,0},         extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=VDC)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={30,-30})));
        Modelica.Blocks.Sources.Ramp ramp(height=pi, duration=10)
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-30,68})));
      equation
        connect(resistor.n, inductor.p) annotation (Line(
            points={{30,20},{30,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(inductor.n, constantVoltage.p) annotation (Line(
            points={{30,-10},{30,-20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(constantVoltage.n, currentSensor.p) annotation (Line(
            points={{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(ramp.y, pulse2m.u) annotation (Line(
            points={{-30,57},{-30,40}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent={{-100,
                  -100},{100,100}},                                                                                                    preserveAspectRatio=false,  initialScale = 0.1, grid = {2,2}),
              graphics),                                                                                                    experiment(
            StopTime=10,
            __Dymola_NumberOfIntervals=50000,
            Tolerance=1e-06),
          __Dymola_experimentSetupOutput);
      end FullBridge2PulseCenterTap_RLV_Characteristic;
      annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})));
    end FullBridge2PulseCenterTap;

    package FullBridge2Pulse "Two pulse Graetz bridge"
      extends Modelica.Icons.ExamplesPackage;
      model FullBridge2Pulse_R
        extends
          Modelica_Electrical_PowerConverters.Examples.ExampleTemplates.FullBridgeTwoPulse(
            pulse2m(alpha=alpha));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle alpha = 90 * pi / 180 "Firing angle";
        parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)    annotation(Placement(visible = true, transformation(origin={30,30},                     extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
      equation
        connect(resistor.n, currentSensor.p) annotation (Line(
            points={{30,20},{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent={{-100,
                  -100},{100,100}},                                                                                                    preserveAspectRatio=false,  initialScale = 0.1, grid = {2,2}),
              graphics),                                                                                                    experiment(
            StopTime=0.1,
            __Dymola_NumberOfIntervals=50000,
            Tolerance=1e-06),
          __Dymola_experimentSetupOutput);
      end FullBridge2Pulse_R;

      model FullBridge2Pulse_RL
        extends
          Modelica_Electrical_PowerConverters.Examples.ExampleTemplates.FullBridgeTwoPulse(
            pulse2m(alpha=alpha));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle alpha = 90 * pi / 180 "Firing angle";
        parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        parameter Modelica.SIunits.Inductance L = 1 "Load resistance" annotation(Evaluate=true);
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)    annotation(Placement(visible = true, transformation(origin={30,30},                     extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L)    annotation(Placement(visible = true, transformation(origin={30,0},         extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
      equation
        connect(resistor.n, inductor.p) annotation (Line(
            points={{30,20},{30,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(inductor.n, currentSensor.p) annotation (Line(
            points={{30,-10},{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent={{-100,
                  -100},{100,100}},                                                                                                    preserveAspectRatio=false,  initialScale = 0.1, grid = {2,2}),
              graphics),                                                                                                    experiment(
            StopTime=0.1,
            __Dymola_NumberOfIntervals=50000,
            Tolerance=1e-06),
          __Dymola_experimentSetupOutput);
      end FullBridge2Pulse_RL;

      model FullBridge2Pulse_RLV
        extends
          Modelica_Electrical_PowerConverters.Examples.ExampleTemplates.FullBridgeTwoPulse(
            pulse2m(alpha=alpha));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle alpha = 90 * pi / 180 "Firing angle";
        parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        parameter Modelica.SIunits.Inductance L = 1 "Load resistance" annotation(Evaluate=true);
        parameter Modelica.SIunits.Voltage VDC=-120 "DC load offset voltage";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)    annotation(Placement(visible = true, transformation(origin={30,30},                     extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L)    annotation(Placement(visible = true, transformation(origin={30,0},         extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=VDC)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={30,-30})));
      equation
        connect(resistor.n, inductor.p) annotation (Line(
            points={{30,20},{30,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(inductor.n, constantVoltage.p) annotation (Line(
            points={{30,-10},{30,-20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(constantVoltage.n, currentSensor.p) annotation (Line(
            points={{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent={{-100,
                  -100},{100,100}},                                                                                                    preserveAspectRatio=false,  initialScale = 0.1, grid = {2,2}),
              graphics),                                                                                                    experiment(
            StopTime=0.1,
            __Dymola_NumberOfIntervals=50000,
            Tolerance=1e-06),
          __Dymola_experimentSetupOutput);
      end FullBridge2Pulse_RLV;

      model FullBridge2Pulse_RLV_Characteristic
        extends
          Modelica_Electrical_PowerConverters.Examples.ExampleTemplates.FullBridgeTwoPulse;
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        parameter Modelica.SIunits.Inductance L = 1 "Load resistance" annotation(Evaluate=true);
        parameter Modelica.SIunits.Voltage VDC=-120 "DC load offset voltage";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)    annotation(Placement(visible = true, transformation(origin={30,30},                     extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L)    annotation(Placement(visible = true, transformation(origin={30,0},         extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=VDC)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={30,-30})));
        Modelica.Blocks.Sources.Ramp ramp(height=pi, duration=10)
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-30,70})));
      equation
        connect(resistor.n, inductor.p) annotation (Line(
            points={{30,20},{30,10},{30,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(inductor.n, constantVoltage.p) annotation (Line(
            points={{30,-10},{30,-20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(constantVoltage.n, currentSensor.p) annotation (Line(
            points={{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(ramp.y, pulse2m.u) annotation (Line(
            points={{-30,59},{-30,40}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent={{-100,
                  -100},{100,100}},                                                                                                    preserveAspectRatio=false,  initialScale = 0.1, grid = {2,2}),
              graphics),                                                                                                    experiment(
            StopTime=10,
            __Dymola_NumberOfIntervals=50000,
            Tolerance=1e-06),
          __Dymola_experimentSetupOutput);
      end FullBridge2Pulse_RLV_Characteristic;
    end FullBridge2Pulse;


    package FullBridge2mPulse
      extends Modelica.Icons.ExamplesPackage;
      model Impulses
        extends Modelica.Icons.Example;
        parameter Integer m(final min=3)=3 "Number of phases";
        Modelica.Electrical.Analog.Basic.Ground ground
          annotation (Placement(transformation(extent={{-90,-60},{-70,-40}})));
        Modelica.Electrical.MultiPhase.Basic.Star star(final m=m)
                                                       annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-80,-10})));
        Modelica.Electrical.MultiPhase.Sources.SineVoltage sineVoltage(
          final m=m,
          V=fill(sqrt(2)*100, m),
          freqHz=fill(1/360, m),
          phase=-Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(
              m))                annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-80,30})));
        Control.Signal2mPulse
                          twomPulse(
          f=1/360,
          final m=m,
          alpha=0.017453292519943)
          annotation (Placement(transformation(extent={{-20,40},{0,60}})));
        Modelica.Electrical.MultiPhase.Basic.Delta deltaP(final m=m)
                                                               annotation (
            Placement(transformation(
              extent={{10,-10},{-10,10}},
              rotation=90,
              origin={-60,50})));
        Modelica.Electrical.MultiPhase.Sensors.VoltageSensor voltageSensorP(final m=m)
          annotation (Placement(transformation(
              extent={{10,10},{-10,-10}},
              rotation=270,
              origin={-40,50})));
      equation
        connect(sineVoltage.plug_n, star.plug_p) annotation (Line(
            points={{-80,20},{-80,0}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(star.pin_n, ground.p) annotation (Line(
            points={{-80,-20},{-80,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sineVoltage.plug_p, deltaP.plug_n) annotation (Line(
            points={{-80,40},{-60,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(deltaP.plug_n, voltageSensorP.plug_p) annotation (Line(
            points={{-60,40},{-40,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(deltaP.plug_p, voltageSensorP.plug_n) annotation (Line(
            points={{-60,60},{-50,60},{-50,60},{-40,60}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(voltageSensorP.v, twomPulse.v) annotation (Line(
            points={{-29,50},{-20,50}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics),
          experiment(StopTime=360),
          __Dymola_experimentSetupOutput);
      end Impulses;

      model FullBridge2mPulse_R
        extends
          Modelica_Electrical_PowerConverters.Examples.ExampleTemplates.FullBridge2mPulse(
            pulse2m(alpha=alpha));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle alpha = 90 * pi / 180 "Firing angle";
        parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)
          annotation(Placement(visible = true, transformation(origin={30,30},                     extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
      equation
        connect(resistor.n, currentSensor.p) annotation (Line(
            points={{30,20},{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics),
          experiment(StopTime=0.1, __Dymola_NumberOfIntervals=5000),
          __Dymola_experimentSetupOutput);
      end FullBridge2mPulse_R;

      model FullBridge2mPulse_RL
        extends
          Modelica_Electrical_PowerConverters.Examples.ExampleTemplates.FullBridge2mPulse(
            pulse2m(alpha=alpha));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle alpha = 90 * pi / 180 "Firing angle";
        parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        parameter Modelica.SIunits.Inductance L = 1 "Load resistance" annotation(Evaluate=true);
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)    annotation(Placement(visible = true, transformation(origin={30,30},                     extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L)    annotation(Placement(visible = true, transformation(origin={30,0},         extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
      equation
        connect(resistor.n, inductor.p) annotation (Line(
            points={{30,20},{30,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(inductor.n, currentSensor.p) annotation (Line(
            points={{30,-10},{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics),
          experiment(StopTime=0.1, __Dymola_NumberOfIntervals=5000),
          __Dymola_experimentSetupOutput);
      end FullBridge2mPulse_RL;

      model FullBridge2mPulse_RLV
        extends
          Modelica_Electrical_PowerConverters.Examples.ExampleTemplates.FullBridge2mPulse(
            pulse2m(alpha=alpha));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle alpha = 90 * pi / 180 "Firing angle";
        parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        parameter Modelica.SIunits.Inductance L = 1 "Load resistance" annotation(Evaluate=true);
        parameter Modelica.SIunits.Voltage VDC=-260 "DC load offset voltage";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)    annotation(Placement(visible = true, transformation(origin={30,30},                     extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L)    annotation(Placement(visible = true, transformation(origin={30,0},         extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=VDC)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={30,-30})));
      equation
        connect(resistor.n, inductor.p) annotation (Line(
            points={{30,20},{30,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(inductor.n, constantVoltage.p) annotation (Line(
            points={{30,-10},{30,-10},{30,-20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(constantVoltage.n, currentSensor.p) annotation (Line(
            points={{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics),
          experiment(StopTime=0.1, __Dymola_NumberOfIntervals=5000),
          __Dymola_experimentSetupOutput);
      end FullBridge2mPulse_RLV;

      model FullBridge2mPulse_RLV_Characteristic
        extends
          Modelica_Electrical_PowerConverters.Examples.ExampleTemplates.FullBridge2mPulse(
            pulse2m(useSignal=true));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        parameter Modelica.SIunits.Inductance L = 1 "Load resistance" annotation(Evaluate=true);
        parameter Modelica.SIunits.Voltage VDC=-260 "DC load offset voltage";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)    annotation(Placement(visible = true, transformation(origin={30,30},                     extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L)    annotation(Placement(visible = true, transformation(origin={30,0},         extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=VDC)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={30,-30})));
        Modelica.Blocks.Sources.Ramp ramp(height=pi, duration=10)
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-30,70})));
      equation
        connect(resistor.n, inductor.p) annotation (Line(
            points={{30,20},{30,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(inductor.n, constantVoltage.p) annotation (Line(
            points={{30,-10},{30,-20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(currentSensor.p, constantVoltage.n) annotation (Line(
            points={{10,-40},{30,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(ramp.y, pulse2m.u) annotation (Line(
            points={{-30,59},{-30,40}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics),
          experiment(StopTime=10, __Dymola_NumberOfIntervals=5000),
          __Dymola_experimentSetupOutput);
      end FullBridge2mPulse_RLV_Characteristic;
    end FullBridge2mPulse;

    package ExampleTemplates
      partial model FullBridge1Pulse
        extends Modelica_Electrical_PowerConverters.Icons.ExampleTemplate;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Voltage Vrms = 110 "RMS supply voltage";
        parameter Modelica.SIunits.Frequency f = 50 "Frequency";
        // parameter Modelica.SIunits.Angle alpha = 90 * pi / 180 "Firing angle";
        // parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        Modelica.Electrical.Analog.Sensors.VoltageSensor voltagesensor  annotation(Placement(visible = true, transformation(origin={50,10},          extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Sources.SineVoltage sinevoltage(V=sqrt(2)*Vrms,
            freqHz=f) annotation (Placement(visible=true, transformation(
              origin={-80,0},
              extent={{-10,-10},{10,10}},
              rotation=-90)));
        Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(visible=true,
              transformation(
              origin={-80,-50},
              extent={{-10,-10},{10,10}},
              rotation=0)));
        Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor
          annotation (Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=180,
              origin={0,-40})));
        Modelica.Blocks.Math.Mean meanVoltage(f=f) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,40})));
        Modelica.Blocks.Math.RootMeanSquare rootMeanSquareVoltage(f=f)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,10})));
        Modelica.Blocks.Math.Mean meanCurrent(f=f) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,-60})));
        Control.Voltage2Pulse                                     twoPulse(f=f)
         annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-40,0})));
        Modelica.Electrical.Analog.Ideal.IdealThyristor idealthyristor annotation (
            Placement(visible=true, transformation(
              origin={-10,40},
              extent={{-10,10},{10,-10}},
              rotation=0)));
      equation
        connect(meanCurrent.u, currentSensor.i) annotation (Line(
            points={{68,-60},{-4.44089e-16,-60},{-4.44089e-16,-50}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(ground.p, sinevoltage.n) annotation (Line(
            points={{-80,-40},{-80,-10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(voltagesensor.v, meanVoltage.u) annotation (Line(
            points={{60,10},{64,10},{64,40},{68,40}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(voltagesensor.v, rootMeanSquareVoltage.u) annotation (Line(
            points={{60,10},{68,10}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(sinevoltage.p, idealthyristor.p) annotation (Line(
            points={{-80,10},{-80,40},{-20,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(ground.p, currentSensor.n) annotation (Line(
            points={{-80,-40},{-10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(twoPulse.firePositive, idealthyristor.fire) annotation (Line(
            points={{-44,-11},{-44,-20},{-3,-20},{-3,29}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(voltagesensor.n, currentSensor.p) annotation (Line(
            points={{50,0},{50,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(idealthyristor.n, voltagesensor.p) annotation (Line(
            points={{0,40},{50,40},{50,20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sinevoltage.p, twoPulse.ac_p) annotation (Line(
            points={{-80,10},{-60,10},{-60,6},{-50,6}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sinevoltage.n, twoPulse.ac_n) annotation (Line(
            points={{-80,-10},{-60,-10},{-60,-6},{-50,-6}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent={{-100,
                  -100},{100,100}},                                                                                                    preserveAspectRatio=false,  initialScale = 0.1, grid = {2,2}),
              graphics),                                                                                                    experiment(
            StopTime=0.1,
            __Dymola_NumberOfIntervals=5000,
            Tolerance=1e-06),
          __Dymola_experimentSetupOutput,
          Documentation(info="<html>
<p>Inductive load does not make sense, since average DC voltage is very low due to long conduction period of the thyristor. </p>
</html>"));
      end FullBridge1Pulse;
      extends Modelica.Icons.Package;
      model FullBridge2PulseCenterTap
        extends Modelica_Electrical_PowerConverters.Icons.ExampleTemplate;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Voltage Vrms = 110 "RMS supply voltage";
        parameter Modelica.SIunits.Frequency f = 50 "Frequency";
        // parameter Modelica.SIunits.Angle alpha = 90 * pi / 180 "Firing angle";
        // parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        // parameter Modelica.SIunits.Inductance L = 1 "Load resistance" annotation(Evaluate=true);
        // parameter Modelica.SIunits.Voltage VDC=-120 "DC load offset voltage";
        Modelica.Electrical.Analog.Basic.Ground ground
          annotation (Placement(visible=true,
              transformation(
              origin={-90,-50},
              extent={{-10,-10},{10,10}},
              rotation=0)));
        Modelica.Electrical.Analog.Sources.SineVoltage sinevoltage2(V = sqrt(2) * Vrms, freqHz = f)
          annotation(Placement(visible = true, transformation(origin={-80,
                  -13.9999},                                                                                                    extent={{
                  -9.99989,-10},{10,10}},                                                                                                    rotation = -90)));
        Modelica.Electrical.Analog.Sources.SineVoltage sinevoltage1(V = sqrt(2) * Vrms, freqHz = f)
          annotation(Placement(visible = true, transformation(origin={-80,14},             extent={{-10,-10},
                  {10,10}},                                                                                                    rotation = -90)));
        Modelica_Electrical_PowerConverters.ACDC.FullBridge2PulseCenterTap rectifier
          annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
        Modelica.Electrical.Analog.Sensors.VoltageSensor voltagesensor
          annotation(Placement(visible = true, transformation(origin={50,10},          extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Blocks.Math.Mean meanVoltage(f=2*f)
          annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,40})));
        Modelica.Blocks.Math.RootMeanSquare rootMeanSquareVoltage(f=2*f)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,10})));
        Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor
          annotation (Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=180,
              origin={0,-40})));
        Modelica.Blocks.Math.Mean meanCurrent(f=2*f)
          annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,-60})));
        Control.Voltage2Pulse pulse2m(f=f) annotation (Placement(transformation(
              extent={{10,10},{-10,-10}},
              rotation=180,
              origin={-30,30})));
      equation
        connect(meanCurrent.u, currentSensor.i) annotation (Line(
            points={{68,-60},{-4.44089e-16,-60},{-4.44089e-16,-50}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(voltagesensor.v, meanVoltage.u) annotation (Line(
            points={{60,10},{64,10},{64,40},{68,40}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(voltagesensor.v, rootMeanSquareVoltage.u) annotation (Line(
            points={{60,10},{68,10}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(sinevoltage1.n, sinevoltage2.p) annotation (Line(
            points={{-80,4},{-80,-4.00001}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(ground.p, sinevoltage1.n) annotation (Line(
            points={{-90,-40},{-90,0},{-80,0},{-80,4}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sinevoltage1.p, rectifier.ac_p)
                                          annotation (Line(
            points={{-80,24},{-80,32},{-60,32},{-60,6},{-40,6}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sinevoltage2.n, rectifier.ac_n)
                                          annotation (Line(
            points={{-80,-23.9999},{-80,-30},{-60,-30},{-60,-6},{-40,-6}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(ground.p, currentSensor.n) annotation (Line(
            points={{-90,-40},{-10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(currentSensor.p, voltagesensor.n) annotation (Line(
            points={{10,-40},{50,-40},{50,0}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(voltagesensor.p, rectifier.dc_p)
                                           annotation (Line(
            points={{50,20},{50,40},{-10,40},{-10,0},{-20,0}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(pulse2m.firePositive, rectifier.firePositive) annotation (Line(
            points={{-34,19},{-34,12}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(pulse2m.fireNegative, rectifier.fireNegative) annotation (Line(
            points={{-26,19},{-26,12}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(rectifier.ac_n, pulse2m.ac_n) annotation (Line(
            points={{-40,-6},{-46,-6},{-46,24},{-40,24}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(pulse2m.ac_p, rectifier.ac_p) annotation (Line(
            points={{-40,36},{-50,36},{-50,6},{-40,6}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent={{-100,
                  -100},{100,100}},                                                                                                    preserveAspectRatio=false,  initialScale = 0.1, grid = {2,2}),
              graphics),                                                                                                    experiment(
            StopTime=10,
            __Dymola_NumberOfIntervals=50000,
            Tolerance=1e-06),
          __Dymola_experimentSetupOutput);
      end FullBridge2PulseCenterTap;

      partial model FullBridgeTwoPulse "Template of B2C without load"
        extends Modelica_Electrical_PowerConverters.Icons.ExampleTemplate;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Voltage Vrms = 110 "RMS supply voltage";
        parameter Modelica.SIunits.Frequency f = 50 "Frequency";
        // parameter Modelica.SIunits.Angle alpha = 90 * pi / 180 "Firing angle";
        // parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        // parameter Modelica.SIunits.Inductance L = 1 "Load resistance" annotation(Evaluate=true);
        // parameter Modelica.SIunits.Voltage VDC=-120 "DC load offset voltage";
        Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(visible=true,
              transformation(
              origin={-80,-50},
              extent={{-10,-10},{10,10}},
              rotation=0)));
        Modelica.Electrical.Analog.Sources.SineVoltage sinevoltage(V=sqrt(2)*Vrms,
            freqHz=f) annotation (Placement(visible=true, transformation(
              origin={-80,0},
              extent={{-10,-10},{10,10}},
              rotation=-90)));
        Modelica_Electrical_PowerConverters.ACDC.HalfBridge2Pulse rectifier(
            useHeatPort=false)
          annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
        Modelica.Electrical.Analog.Sensors.VoltageSensor voltagesensor  annotation(Placement(visible = true, transformation(origin={50,10},          extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Blocks.Math.Mean meanVoltage(f=2*f)
          annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,40})));
        Modelica.Blocks.Math.RootMeanSquare rootMeanSquareVoltage(f=2*f)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,10})));
        Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor
          annotation (Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=180,
              origin={0,-40})));
        Modelica.Blocks.Math.Mean meanCurrent(f=2*f)
          annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,-60})));
        Control.Voltage2Pulse pulse2m(f=f) annotation (Placement(transformation(
              extent={{10,10},{-10,-10}},
              rotation=180,
              origin={-30,30})));
      equation
        connect(meanCurrent.u, currentSensor.i) annotation (Line(
            points={{68,-60},{-4.44089e-16,-60},{-4.44089e-16,-50}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(ground.p, sinevoltage.n) annotation (Line(
            points={{-80,-40},{-80,-10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(voltagesensor.v, meanVoltage.u) annotation (Line(
            points={{60,10},{64,10},{64,40},{68,40}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(voltagesensor.v, rootMeanSquareVoltage.u) annotation (Line(
            points={{60,10},{68,10}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(sinevoltage.p, rectifier.ac_p)
                                         annotation (Line(
            points={{-80,10},{-62,10},{-62,6},{-40,6}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sinevoltage.n, rectifier.ac_n)
                                         annotation (Line(
            points={{-80,-10},{-62,-10},{-62,-6},{-40,-6}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(rectifier.dc_n, currentSensor.n)
                                           annotation (Line(
            points={{-19.8,-6},{-10,-6},{-10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(rectifier.dc_p, voltagesensor.p)
                                           annotation (Line(
            points={{-20,6},{-10,6},{-10,40},{50,40},{50,20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(voltagesensor.n, currentSensor.p) annotation (Line(
            points={{50,0},{50,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(pulse2m.firePositive, rectifier.firePositive) annotation (Line(
            points={{-34,19},{-34,12}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(pulse2m.fireNegative, rectifier.fireNegative) annotation (Line(
            points={{-26,19},{-26,12}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(pulse2m.ac_p, rectifier.ac_p) annotation (Line(
            points={{-40,36},{-60,36},{-60,6},{-40,6}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(rectifier.ac_n, pulse2m.ac_n) annotation (Line(
            points={{-40,-6},{-56,-6},{-56,24},{-40,24}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent={{-100,
                  -100},{100,100}},                                                                                                    preserveAspectRatio=false,  initialScale = 0.1, grid = {2,2}),
              graphics),                                                                                                    experiment(
            StopTime=10,
            __Dymola_NumberOfIntervals=50000,
            Tolerance=1e-06),
          __Dymola_experimentSetupOutput);
      end FullBridgeTwoPulse;


      partial model FullBridge2mPulse "Template of B2*mC without load"
        extends Modelica_Electrical_PowerConverters.Icons.ExampleTemplate;
        import Modelica.Constants.pi;
        parameter Integer m(final min=3)=3 "Number of phases";
        parameter Modelica.SIunits.Voltage Vrms = 110 "RMS supply voltage";
        parameter Modelica.SIunits.Frequency f = 50 "Frequency";
        // parameter Modelica.SIunits.Angle alpha = 90 * pi / 180 "Firing angle";
        // parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        // parameter Modelica.SIunits.Inductance L = 1 "Load resistance" annotation(Evaluate=true);
        // parameter Modelica.SIunits.Voltage VDC=-260 "DC load offset voltage";
        Modelica.Electrical.Analog.Basic.Ground ground
          annotation (Placement(transformation(extent={{-90,-80},{-70,-60}})));
        Modelica.Electrical.MultiPhase.Basic.Star star(final m=m)
                                                       annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-80,-40})));
        Modelica.Electrical.MultiPhase.Sources.SineVoltage sineVoltage(
          final m=m,
          V=fill(sqrt(2)*Vrms, m),
          phase=-Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(
              m),
          freqHz=fill(f, m))
                  annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-80,-10})));
        ACDC.FullBridge2mPulse rectifier(final m=m)
          annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
        Modelica.Electrical.Analog.Sensors.VoltageSensor voltagesensor
          annotation(Placement(visible = true, transformation(origin={50,10},
            extent={{10,-10},{-10,10}},                                                                                                    rotation=90)));
        Modelica.Blocks.Math.Mean meanVoltage(f=2*m*f)
          annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,40})));
        Modelica.Blocks.Math.RootMeanSquare rootMeanSquareVoltage(f=2*m*f)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,10})));
        Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor
          annotation (Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=180,
              origin={0,-40})));
        Modelica.Blocks.Math.Mean meanCurrent(f=2*m*f)
          annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,-60})));
        Control.Voltage2mPulse pulse2m(m=m, f=f) annotation (Placement(
              transformation(
              extent={{10,10},{-10,-10}},
              rotation=180,
              origin={-30,30})));
      equation
        connect(sineVoltage.plug_n, star.plug_p) annotation (Line(
            points={{-80,-20},{-80,-30}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(star.pin_n, ground.p) annotation (Line(
            points={{-80,-50},{-80,-60}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(meanCurrent.u,currentSensor. i) annotation (Line(
            points={{68,-60},{0,-60},{0,-50}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(voltagesensor.v,meanVoltage. u) annotation (Line(
            points={{60,10},{64,10},{64,40},{68,40}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(voltagesensor.v,rootMeanSquareVoltage. u) annotation (Line(
            points={{60,10},{68,10}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(sineVoltage.plug_p, rectifier.ac)
                                            annotation (Line(
            points={{-80,0},{-40,0}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(rectifier.dc_n, currentSensor.n)
                                           annotation (Line(
            points={{-19.8,-6},{-10,-6},{-10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(rectifier.dc_p, voltagesensor.p)
                                           annotation (Line(
            points={{-20,6},{-10,6},{-10,40},{50,40},{50,20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(currentSensor.p, voltagesensor.n) annotation (Line(
            points={{10,-40},{50,-40},{50,0}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(pulse2m.firePositive, rectifier.firePositive) annotation (Line(
            points={{-34,19},{-34,12}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(pulse2m.fireNegative, rectifier.fireNegative) annotation (Line(
            points={{-26,19},{-26,12}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(pulse2m.ac, sineVoltage.plug_p) annotation (Line(
            points={{-40,30},{-80,30},{-80,4.44089e-16}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics),
          experiment(StopTime=0.04, __Dymola_NumberOfIntervals=5000),
          __Dymola_experimentSetupOutput,
          Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
              graphics));
      end FullBridge2mPulse;
    end ExampleTemplates;
  end Examples;

  package Control "Control components for rectifiers"
    extends Modelica.Icons.Package;

    block Signal2mPulse "Boolean impulses for 2*m pulse rectifiers"
      import Modelica.Constants.pi;
      parameter Integer m(final min=1)=3 "Number of phases";
      parameter Boolean useSignal = false
        "Enables signal input instead of parameter";
      parameter Modelica.SIunits.Frequency f = 50 "Frequency";
      parameter Modelica.SIunits.Angle alpha = 0 "Firing angle"
        annotation (Dialog(enable=not useSignal));
      parameter Modelica.SIunits.Angle alphaMax(min=0,max=Modelica.Constants.pi) = Modelica.Constants.pi
        "Maximum firing angle";
      Modelica.Blocks.Interfaces.RealInput u if useSignal
        annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={0,100})));
      Modelica.Blocks.Sources.Constant constantalpha(final k=alpha) if not useSignal
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-30,90})));
      Modelica.Blocks.Logical.GreaterThreshold positiveThreshold[m](threshold=zeros(m)) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-40,-2})));
      Modelica.Blocks.Logical.LessThreshold negativeThreshold[m](threshold=zeros(m)) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={40,0})));
      Modelica.Blocks.Logical.Timer timerPositive[m] annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-40,-32})));
      Modelica.Blocks.Logical.Timer timerNegative[m] annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={40,-32})));
      Modelica.Blocks.Logical.Greater greaterPositive[m] annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-40,-72})));
      Modelica.Blocks.Logical.Greater negativeEqual[m] annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={40,-72})));
      Modelica.Blocks.Interfaces.BooleanOutput firePositive[m] annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-40,-110})));
      Modelica.Blocks.Interfaces.BooleanOutput fireNegative[m] annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={40,-110})));
      Modelica.Blocks.Math.Gain gain(final k=1/2/pi/f)
        annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,0})));
      Modelica.Blocks.Routing.Replicator replicator(final nout=m)
        annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,-30})));
      Modelica.Blocks.Nonlinear.Limiter limiter(final uMax=max(Modelica.Constants.pi, alphaMax), final uMin=0)
        annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,50})));
      Modelica.Blocks.Interfaces.RealInput v[m]
        annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={-100,0})));
    equation
      connect(positiveThreshold.y, timerPositive.u)
                                                  annotation (Line(
          points={{-40,-13},{-40,-14},{-40,-14},{-40,-16},{-40,-16},{-40,-20}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(negativeThreshold.y, timerNegative.u)
                                            annotation (Line(
          points={{40,-11},{40,-20}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(timerPositive.y, greaterPositive.u1) annotation (Line(
          points={{-40,-43},{-40,-48},{-42,-48},{-42,-52},{-40,-52},{-40,-60}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(negativeEqual.u1, timerNegative.y) annotation (Line(
          points={{40,-60},{40,-43}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(greaterPositive.y, firePositive) annotation (Line(
          points={{-40,-83},{-40,-110}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(negativeEqual.y, fireNegative)
                                      annotation (Line(
          points={{40,-83},{40,-88},{40,-88},{40,-96},{40,-96},{40,-110},{40,
              -110}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(gain.y, replicator.u) annotation (Line(
          points={{-2.22045e-015,-11},{-2.22045e-015,-18},{2.22045e-015,-18}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(replicator.y, greaterPositive.u2) annotation (Line(
          points={{-1.33227e-15,-41},{-1.33227e-15,-50},{-48,-50},{-48,-60}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(replicator.y, negativeEqual.u2) annotation (Line(
          points={{-1.33227e-15,-41},{-1.33227e-15,-50},{32,-50},{32,-60}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(limiter.y, gain.u) annotation (Line(
          points={{-1.9984e-015,39},{-1.9984e-015,12},{2.22045e-015,12}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(u, limiter.u) annotation (Line(
          points={{8.88178e-016,100},{0,100},{0,70},{2.22045e-015,70},{2.22045e-015,
              62}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(constantalpha.y, limiter.u) annotation (Line(
          points={{-30,79},{-30,70},{0,70},{0,62},{2.22045e-015,62}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(v, positiveThreshold.u) annotation (Line(
          points={{-100,1.11022e-15},{-80,1.11022e-15},{-80,0},{-60,0},{-60,20},
              {-40,20},{-40,10}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(v, negativeThreshold.u) annotation (Line(
          points={{-100,8.88178e-16},{-60,8.88178e-16},{-60,20},{40,20},{40,12}},
          color={0,0,127},
          smooth=Smooth.None));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics), Icon(coordinateSystem(
              preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
            graphics={
            Rectangle(extent={{-100,100},{100,-100}}, lineColor={0,0,255}),
            Line(
              points={{-40,-20},{-40,-24},{-20,-24},{-20,-40},{-40,-40},{-40,-60}},
              color={255,0,255},
              smooth=Smooth.None),
            Line(
              points={{20,-20},{20,-44},{40,-44},{40,-60},{20,-60},{20,-60}},
              color={255,0,255},
              smooth=Smooth.None),
            Text(
              extent={{-40,60},{40,0}},
              lineColor={255,0,255},
              fillColor={0,0,255},
              fillPattern=FillPattern.Solid,
              textString="2*%m%")}));
    end Signal2mPulse;


    model Voltage2Pulse "Two pulse Graetz full bridge SCR"
      import Modelica.Constants.pi;
      parameter Boolean useSignal = false
        "Enables signal input instead of parameter";
      parameter Modelica.SIunits.Frequency f = 50 "Frequency";
      parameter Modelica.SIunits.Angle alpha = 0 "Firing angle"
        annotation (Dialog(enable=not useSignal));
      parameter Modelica.SIunits.Angle alphaMax(min=0,max=Modelica.Constants.pi) = Modelica.Constants.pi
        "Maximum firing angle";
      parameter Modelica.SIunits.Resistance Ron(final min=0) = 1.E-5
        "Closed thyristor resistance";
      parameter Modelica.SIunits.Conductance Goff(final min=0) = 1.E-5
        "Opened thyristor conductance";
      parameter Modelica.SIunits.Voltage Vknee(final min=0) = 0
        "Forward threshold voltage";

      Modelica.Blocks.Interfaces.RealInput u if useSignal
        annotation (Placement(transformation(extent={{-20,-20},{20,20}},
            rotation=270,
            origin={0,100})));
      Signal2mPulse                   twoPulse(
        final useSignal=useSignal,
        final f=f,
        final alpha=alpha,
        final alphaMax=alphaMax,
        final m=1)
       annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={-30,0})));
      Modelica.Electrical.Analog.Interfaces.PositivePin ac_p
        annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin ac_n
        annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
      Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor
        annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=270,
            origin={-80,30})));
      Modelica.Blocks.Interfaces.BooleanOutput firePositive    annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-40,-110})));
      Modelica.Blocks.Interfaces.BooleanOutput fireNegative    annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={40,-110})));
    equation

      connect(twoPulse.u, u) annotation (Line(
          points={{-30,10},{-30,60},{0,60},{0,100},{1.11022e-15,100}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(voltageSensor.v, twoPulse.v[1]) annotation (Line(
          points={{-70,30},{-60,30},{-60,0},{-40,0},{-40,6.66134e-16}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(voltageSensor.p, ac_p) annotation (Line(
          points={{-80,40},{-80,60},{-100,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(voltageSensor.n, ac_n) annotation (Line(
          points={{-80,20},{-80,-60},{-100,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(twoPulse.firePositive[1], firePositive) annotation (Line(
          points={{-34,-11},{-34,-60},{-40,-60},{-40,-110}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(twoPulse.fireNegative[1], fireNegative) annotation (Line(
          points={{-26,-11},{-26,-60},{40,-60},{40,-110}},
          color={255,0,255},
          smooth=Smooth.None));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics), Icon(coordinateSystem(
              preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
            graphics={
            Rectangle(
              extent={{-100,100},{100,-100}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
                                            Text(
            extent={{-150,150},{150,110}},
            textString="%name",
            lineColor={0,0,255}),
            Line(
              points={{-40,-20},{-40,-24},{-20,-24},{-20,-40},{-40,-40},{-40,
                  -60}},
              color={255,0,255},
              smooth=Smooth.None),
            Line(
              points={{20,-20},{20,-44},{40,-44},{40,-60},{20,-60},{20,-60}},
              color={255,0,255},
              smooth=Smooth.None),
            Text(
              extent={{-40,60},{40,0}},
              lineColor={255,0,255},
              fillColor={0,0,255},
              fillPattern=FillPattern.Solid,
              textString="2")}));
    end Voltage2Pulse;

    model Voltage2mPulse "2*m pulse full bridge SCR"
      import Modelica.Constants.pi;
      parameter Integer m(final min=3)=3 "Number of phases";
      parameter Boolean useSignal = false
        "Enables signal input instead of parameter";
      parameter Modelica.SIunits.Frequency f = 50 "Frequency";
      parameter Modelica.SIunits.Angle alpha = 0 "Firing angle"
        annotation (Dialog(enable=not useSignal));
      parameter Modelica.SIunits.Angle alphaMax(min=0,max=Modelica.Constants.pi) = Modelica.Constants.pi
        "Maximum firing angle";
      parameter Modelica.SIunits.Resistance Ron(final min=0) = 1.E-5
        "Closed thyristor resistance";
      parameter Modelica.SIunits.Conductance Goff(final min=0) = 1.E-5
        "Opened thyristor conductance";
      parameter Modelica.SIunits.Voltage Vknee(final min=0) = 0
        "Forward threshold voltage";

      Modelica.Blocks.Interfaces.RealInput u if useSignal
        annotation (Placement(transformation(extent={{-20,-20},{20,20}},
            rotation=270,
            origin={0,100})));
      Signal2mPulse                   twomPulse(
        final useSignal=useSignal,
        final f=f,
        final alpha=alpha,
        final alphaMax=alphaMax,
        final m=m)         annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={-30,0})));
      Modelica.Electrical.MultiPhase.Interfaces.PositivePlug ac(final m=m)
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
      Modelica.Electrical.MultiPhase.Basic.Delta deltaP(final m=m)
                                                             annotation (
          Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=0,
            origin={-70,70})));
      Modelica.Electrical.MultiPhase.Sensors.VoltageSensor voltageSensorP(final m=m)
        annotation (Placement(transformation(
            extent={{10,10},{-10,-10}},
            rotation=180,
            origin={-70,40})));
      Modelica.Blocks.Interfaces.BooleanOutput firePositive[m] annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-40,-110})));
      Modelica.Blocks.Interfaces.BooleanOutput fireNegative[m] annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={40,-110})));
    equation

      connect(twomPulse.u, u)
                             annotation (Line(
          points={{-30,10},{-30,28},{-30,28},{-30,28},{-30,60},{0,60},{0,100},{
              8.88178e-16,100}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(ac, voltageSensorP.plug_p) annotation (Line(
          points={{-100,4.44089e-16},{-100,40},{-80,40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(voltageSensorP.plug_p, deltaP.plug_n) annotation (Line(
          points={{-80,40},{-80,70}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(deltaP.plug_p, voltageSensorP.plug_n) annotation (Line(
          points={{-60,70},{-60,40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(voltageSensorP.v, twomPulse.v) annotation (Line(
          points={{-70,29},{-70,0},{-44,0},{-44,0},{-40,0},{-40,0}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(twomPulse.fireNegative, fireNegative) annotation (Line(
          points={{-26,-11},{-26,-60},{40,-60},{40,-110}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(twomPulse.firePositive, firePositive) annotation (Line(
          points={{-34,-11},{-34,-60},{-40,-60},{-40,-110}},
          color={255,0,255},
          smooth=Smooth.None));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics), Icon(coordinateSystem(
              preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
            graphics={
            Rectangle(
              extent={{-100,100},{100,-100}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
                                            Text(
            extent={{-150,150},{150,110}},
            textString="%name",
            lineColor={0,0,255}),
            Line(
              points={{-40,-20},{-40,-24},{-20,-24},{-20,-40},{-40,-40},{-40,
                  -60}},
              color={255,0,255},
              smooth=Smooth.None),
            Line(
              points={{20,-20},{20,-44},{40,-44},{40,-60},{20,-60},{20,-60}},
              color={255,0,255},
              smooth=Smooth.None),
            Text(
              extent={{-40,60},{40,0}},
              lineColor={255,0,255},
              fillColor={0,0,255},
              fillPattern=FillPattern.Solid,
              textString="2*%m%")}));
    end Voltage2mPulse;
  end Control;

  package ACDC "AC to DC and DC to AC converter"
    extends Modelica.Icons.Package;
    model DiodeBridge2PulseCenterTap
      "Two pulse diode rectifier with center tap"
      import Modelica.Constants.pi;
      parameter Modelica.SIunits.Resistance Ron(final min=0) = 1.E-5
        "Closed thyristor resistance";
      parameter Modelica.SIunits.Conductance Goff(final min=0) = 1.E-5
        "Opened thyristor conductance";
      parameter Modelica.SIunits.Voltage Vknee(final min=0) = 0
        "Forward threshold voltage";
      extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(
        final T=293.15);
      Modelica.Electrical.Analog.Ideal.IdealDiode idealDiodeP(
        final Ron=Ron,
        final Goff=Goff,
        final Vknee=Vknee,
        final useHeatPort=useHeatPort) annotation (Placement(visible=true,
            transformation(
            origin={10,60},
            extent={{-10,10},{10,-10}},
            rotation=0)));
      Modelica.Electrical.Analog.Ideal.IdealDiode idealDiodeN(
        final Ron=Ron,
        final Goff=Goff,
        final Vknee=Vknee,
        final useHeatPort=useHeatPort) annotation (Placement(visible=true,
            transformation(
            origin={10,-60},
            extent={{-10,-10},{10,10}},
            rotation=0)));
      Modelica.Electrical.Analog.Interfaces.PositivePin ac_p
        annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin ac_n
        annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
      Modelica.Electrical.Analog.Interfaces.PositivePin dc_p
        annotation (Placement(transformation(extent={{90,-10},{110,10}})));
    equation
      if not useHeatPort then
        LossPower = idealDiodeP.LossPower + idealDiodeN.LossPower;
      end if;
      connect(ac_p, idealDiodeP.p) annotation (Line(
          points={{-100,60},{0,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(ac_n, idealDiodeN.p) annotation (Line(
          points={{-100,-60},{0,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealDiodeP.n, dc_p) annotation (Line(
          points={{20,60},{100,60},{100,0}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealDiodeN.n, dc_p) annotation (Line(
          points={{20,-60},{100,-60},{100,4.44089e-16}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealDiodeN.heatPort, heatPort) annotation (Line(
          points={{10,-70},{10,-100},{0,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(idealDiodeP.heatPort, heatPort) annotation (Line(
          points={{10,70},{30,70},{30,-100},{4.44089e-16,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2}),
            graphics={
            Rectangle(
              extent={{-100,100},{100,-100}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Line(
              points={{-100,-100},{100,100}},
              color={0,0,127},
              smooth=Smooth.None),
            Text(
              extent={{-100,70},{0,50}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="AC"),
            Text(
              extent={{0,-50},{100,-70}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="DC"),
            Text(
              extent={{-40,10},{40,-10}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="M2C")}),                                                                                                    Diagram(coordinateSystem(extent={{-100,
                -100},{100,100}},                                                                                                    preserveAspectRatio=false,  initialScale = 0.1, grid = {2,2}),
            graphics),                                                                                                    experiment(StartTime = 0, StopTime = 0.1, Tolerance = 0.000001));
    end DiodeBridge2PulseCenterTap;

    model FullBridge2PulseCenterTap "Two pulse full bridge SCR with center tap"
      import Modelica.Constants.pi;
      parameter Modelica.SIunits.Resistance Ron(final min=0) = 1.E-5
        "Closed thyristor resistance";
      parameter Modelica.SIunits.Conductance Goff(final min=0) = 1.E-5
        "Opened thyristor conductance";
      parameter Modelica.SIunits.Voltage Vknee(final min=0) = 0
        "Forward threshold voltage";
      extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(
        final T=293.15);
      Modelica.Electrical.Analog.Ideal.IdealThyristor idealThyristorP(
        final Ron=Ron,
        final Goff=Goff,
        final Vknee=Vknee,
        final useHeatPort=useHeatPort)
        annotation(Placement(visible = true, transformation(origin={10,60},
          extent={{-10,10},{10,-10}},                                                                                                    rotation = 0)));
      Modelica.Electrical.Analog.Ideal.IdealThyristor idealThyristorN(
        final Ron=Ron,
        final Goff=Goff,
        final Vknee=Vknee,
        final useHeatPort=useHeatPort)
        annotation(Placement(visible = true, transformation(origin={10,-60},
          extent={{-10,-10},{10,10}},                                                                                                    rotation = 0)));
      Modelica.Electrical.Analog.Interfaces.PositivePin ac_p
        annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin ac_n
        annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
      Modelica.Electrical.Analog.Interfaces.PositivePin dc_p
        annotation (Placement(transformation(extent={{90,-10},{110,10}})));
      Modelica.Blocks.Interfaces.BooleanInput firePositive
        "Fire signal for positive potential semiconductors" annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={-40,120})));
      Modelica.Blocks.Interfaces.BooleanInput fireNegative
        "Fire signal for negative potential semiconductors" annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={40,120})));
    equation
      if not useHeatPort then
        LossPower = idealThyristorP.LossPower + idealThyristorN.LossPower;
      end if;
      connect(ac_p,idealThyristorP. p) annotation (Line(
          points={{-100,60},{-4.44089e-16,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(ac_n,idealThyristorN. p) annotation (Line(
          points={{-100,-60},{-4.44089e-16,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealThyristorP.n, dc_p) annotation (Line(
          points={{20,60},{100,60},{100,0}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealThyristorN.n, dc_p) annotation (Line(
          points={{20,-60},{100,-60},{100,4.44089e-16}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealThyristorN.heatPort, heatPort) annotation (Line(
          points={{10,-70},{10,-100},{0,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(idealThyristorP.heatPort, heatPort) annotation (Line(
          points={{10,70},{10,80},{30,80},{30,-100},{4.44089e-16,-100}},
          color={191,0,0},
          smooth=Smooth.None));

      connect(firePositive, idealThyristorP.fire) annotation (Line(
          points={{-40,120},{-40,40},{17,40},{17,49}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(fireNegative, idealThyristorN.fire) annotation (Line(
          points={{40,120},{40,-40},{17,-40},{17,-49}},
          color={255,0,255},
          smooth=Smooth.None));
      annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2}),
            graphics={
            Rectangle(
              extent={{-100,100},{100,-100}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Line(
              points={{-100,-100},{100,100}},
              color={0,0,127},
              smooth=Smooth.None),
            Text(
              extent={{-100,70},{0,50}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="AC"),
            Text(
              extent={{0,-50},{100,-70}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="DC"),
            Text(
              extent={{-40,10},{40,-10}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="M2C")}),                                                                                                    Diagram(coordinateSystem(extent={{-100,
                -100},{100,100}},                                                                                                    preserveAspectRatio=false,  initialScale = 0.1, grid = {2,2}),
            graphics),                                                                                                    experiment(StartTime = 0, StopTime = 0.1, Tolerance = 0.000001));
    end FullBridge2PulseCenterTap;

    model DiodeBridge2Pulse "Two pulse Graetz diode rectifier"
      import Modelica.Constants.pi;
      parameter Modelica.SIunits.Resistance Ron(final min=0) = 1.E-5
        "Closed thyristor resistance";
      parameter Modelica.SIunits.Conductance Goff(final min=0) = 1.E-5
        "Opened thyristor conductance";
      parameter Modelica.SIunits.Voltage Vknee(final min=0) = 0
        "Forward threshold voltage";
      extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(
         final T=293.15);
      Modelica.Electrical.Analog.Ideal.IdealDiode idealDiodeP1(
        Ron=Ron,
        Goff=Goff,
        Vknee=Vknee,
        useHeatPort=useHeatPort) annotation (Placement(visible=true, transformation(
            origin={10,50},
            extent={{-10,10},{10,-10}},
            rotation=90)));
      Modelica.Electrical.Analog.Ideal.IdealDiode idealDiodeP2(
        Ron=Ron,
        Goff=Goff,
        Vknee=Vknee,
        useHeatPort=useHeatPort) annotation (Placement(visible=true, transformation(
            origin={40,50},
            extent={{-10,10},{10,-10}},
            rotation=90)));
      Modelica.Electrical.Analog.Ideal.IdealDiode idealDiodeN1(
        Ron=Ron,
        Goff=Goff,
        Vknee=Vknee,
        useHeatPort=useHeatPort) annotation (Placement(visible=true, transformation(
            origin={10,-50},
            extent={{-10,-10},{10,10}},
            rotation=90)));
      Modelica.Electrical.Analog.Ideal.IdealDiode idealDiodeN2(
        Ron=Ron,
        Goff=Goff,
        Vknee=Vknee,
        useHeatPort=useHeatPort) annotation (Placement(visible=true, transformation(
            origin={40,-50},
            extent={{-10,-10},{10,10}},
            rotation=90)));
      Modelica.Electrical.Analog.Interfaces.PositivePin ac_p
        annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin ac_n
        annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
      Modelica.Electrical.Analog.Interfaces.PositivePin dc_p
        annotation (Placement(transformation(extent={{90,50},{110,70}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin dc_n
        annotation (Placement(transformation(extent={{92,-70},{112,-50}})));
    equation
      if not useHeatPort then
        LossPower = idealDiodeP1.LossPower + idealDiodeP2.LossPower + idealDiodeN1.LossPower + idealDiodeN2.LossPower;
      end if;
      connect(idealDiodeP2.n, idealDiodeP1.n) annotation (Line(
          points={{40,60},{10,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealDiodeN1.p, idealDiodeN2.p) annotation (Line(
          points={{10,-60},{40,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealDiodeN2.n, idealDiodeP2.p) annotation (Line(
          points={{40,-40},{40,40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealDiodeP1.p, idealDiodeN1.n) annotation (Line(
          points={{10,40},{10,-40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealDiodeP1.n, dc_p) annotation (Line(
          points={{10,60},{100,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealDiodeN1.p, dc_n) annotation (Line(
          points={{10,-60},{102,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealDiodeN1.heatPort, heatPort) annotation (Line(
          points={{20,-50},{20,-100},{4.44089e-16,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(idealDiodeN2.heatPort, heatPort) annotation (Line(
          points={{50,-50},{50,-100},{4.44089e-16,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(idealDiodeP1.heatPort, heatPort) annotation (Line(
          points={{0,50},{-50,50},{-50,-100},{4.44089e-16,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(idealDiodeP2.heatPort, heatPort) annotation (Line(
          points={{30,50},{30,30},{-50,30},{-50,-100},{4.44089e-16,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(ac_p, idealDiodeP1.p) annotation (Line(
          points={{-100,60},{-60,60},{-60,20},{10,20},{10,40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(ac_n, idealDiodeP2.p) annotation (Line(
          points={{-100,-60},{-60,-60},{-60,-20},{40,-20},{40,40}},
          color={0,0,255},
          smooth=Smooth.None));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics), Icon(coordinateSystem(
              preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
            graphics={
            Rectangle(
              extent={{-100,100},{100,-100}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Line(
              points={{-100,-100},{100,100}},
              color={0,0,127},
              smooth=Smooth.None),
            Text(
              extent={{-100,70},{0,50}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="AC"),
            Text(
              extent={{0,-50},{100,-70}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="DC"),             Text(
            extent={{-150,150},{150,110}},
            textString="%name",
            lineColor={0,0,255}),
            Text(
              extent={{-40,10},{40,-10}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="B2C")}));
    end DiodeBridge2Pulse;

    model HalfBridge2Pulse "Two pulse Graetz half bridge SCR"
      import Modelica.Constants.pi;
      parameter Modelica.SIunits.Resistance Ron(final min=0) = 1.E-5
        "Closed thyristor resistance";
      parameter Modelica.SIunits.Conductance Goff(final min=0) = 1.E-5
        "Opened thyristor conductance";
      parameter Modelica.SIunits.Voltage Vknee(final min=0) = 0
        "Forward threshold voltage";
      extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(
         final T=293.15);
      Modelica.Electrical.Analog.Ideal.IdealThyristor idealThyristorP1(
        Ron=Ron,
        Goff=Goff,
        Vknee=Vknee,
        useHeatPort=useHeatPort) annotation (Placement(visible=true, transformation(
            origin={10,50},
            extent={{-10,10},{10,-10}},
            rotation=90)));
      Modelica.Electrical.Analog.Ideal.IdealThyristor idealThyristorP2(
        Ron=Ron,
        Goff=Goff,
        Vknee=Vknee,
        useHeatPort=useHeatPort) annotation (Placement(visible=true, transformation(
            origin={40,50},
            extent={{-10,10},{10,-10}},
            rotation=90)));
      Modelica.Electrical.Analog.Ideal.IdealDiode idealDiodeN1(
        Ron=Ron,
        Goff=Goff,
        Vknee=Vknee,
        useHeatPort=useHeatPort) annotation (Placement(visible=true, transformation(
            origin={10,-50},
            extent={{-10,-10},{10,10}},
            rotation=90)));
      Modelica.Electrical.Analog.Ideal.IdealDiode idealDiodeN2(
        Ron=Ron,
        Goff=Goff,
        Vknee=Vknee,
        useHeatPort=useHeatPort) annotation (Placement(visible=true, transformation(
            origin={40,-50},
            extent={{-10,-10},{10,10}},
            rotation=90)));
      Modelica.Electrical.Analog.Interfaces.PositivePin ac_p
        annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin ac_n
        annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
      Modelica.Electrical.Analog.Interfaces.PositivePin dc_p
        annotation (Placement(transformation(extent={{90,50},{110,70}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin dc_n
        annotation (Placement(transformation(extent={{92,-70},{112,-50}})));
      Modelica.Blocks.Interfaces.BooleanInput firePositive
        "Fire signal for positive potential semiconductors" annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={-40,120})));
      Modelica.Blocks.Interfaces.BooleanInput fireNegative
        "Fire signal for negative potential semiconductors" annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={40,120})));
    equation
      if not useHeatPort then
        LossPower = idealThyristorP1.LossPower + idealThyristorP2.LossPower + idealDiodeN1.LossPower + idealDiodeN2.LossPower;
      end if;
      connect(idealThyristorP2.n,idealThyristorP1. n) annotation (Line(
          points={{40,60},{10,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealDiodeN1.p, idealDiodeN2.p)         annotation (Line(
          points={{10,-60},{40,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealDiodeN2.n, idealThyristorP2.p)     annotation (Line(
          points={{40,-40},{40,40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealThyristorP1.p, idealDiodeN1.n)     annotation (Line(
          points={{10,40},{10,-40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealThyristorP1.n, dc_p) annotation (Line(
          points={{10,60},{100,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealDiodeN1.p, dc_n)     annotation (Line(
          points={{10,-60},{102,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealDiodeN1.heatPort, heatPort)     annotation (Line(
          points={{20,-50},{20,-100},{4.44089e-16,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(idealDiodeN2.heatPort, heatPort)     annotation (Line(
          points={{50,-50},{50,-100},{4.44089e-16,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(idealThyristorP1.heatPort, heatPort) annotation (Line(
          points={{0,50},{-50,50},{-50,-100},{4.44089e-16,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(idealThyristorP2.heatPort, heatPort) annotation (Line(
          points={{30,50},{30,30},{-50,30},{-50,-100},{4.44089e-16,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(ac_p, idealThyristorP1.p) annotation (Line(
          points={{-100,60},{-80,60},{-80,20},{10,20},{10,40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(ac_n, idealDiodeN2.n) annotation (Line(
          points={{-100,-60},{-80,-60},{-80,-20},{40,-20},{40,-40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(firePositive, idealThyristorP1.fire) annotation (Line(
          points={{-40,120},{-40,80},{26,80},{26,57},{21,57}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(idealThyristorP2.fire, fireNegative) annotation (Line(
          points={{51,57},{56,57},{56,80},{40,80},{40,120}},
          color={255,0,255},
          smooth=Smooth.None));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics), Icon(coordinateSystem(
              preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
            graphics={
            Rectangle(
              extent={{-100,100},{100,-100}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Line(
              points={{-100,-100},{100,100}},
              color={0,0,127},
              smooth=Smooth.None),
            Text(
              extent={{-100,70},{0,50}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="AC"),
            Text(
              extent={{0,-50},{100,-70}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="DC"),             Text(
            extent={{-150,150},{150,110}},
            textString="%name",
            lineColor={0,0,255}),
            Text(
              extent={{-40,10},{40,-10}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="B2C")}));
    end HalfBridge2Pulse;

    model FullBridge2Pulse "Two pulse Graetz full bridge SCR"
      import Modelica.Constants.pi;
      parameter Modelica.SIunits.Resistance Ron(final min=0) = 1.E-5
        "Closed thyristor resistance";
      parameter Modelica.SIunits.Conductance Goff(final min=0) = 1.E-5
        "Opened thyristor conductance";
      parameter Modelica.SIunits.Voltage Vknee(final min=0) = 0
        "Forward threshold voltage";
      extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(
         final T=293.15);
      Modelica.Electrical.Analog.Ideal.IdealThyristor idealThyristorP1(
        Ron=Ron,
        Goff=Goff,
        Vknee=Vknee,
        useHeatPort=useHeatPort) annotation (Placement(visible=true, transformation(
            origin={10,50},
            extent={{-10,10},{10,-10}},
            rotation=90)));
      Modelica.Electrical.Analog.Ideal.IdealThyristor idealThyristorP2(
        Ron=Ron,
        Goff=Goff,
        Vknee=Vknee,
        useHeatPort=useHeatPort) annotation (Placement(visible=true, transformation(
            origin={40,50},
            extent={{-10,10},{10,-10}},
            rotation=90)));
      Modelica.Electrical.Analog.Ideal.IdealThyristor idealThyristorN1(
        Ron=Ron,
        Goff=Goff,
        Vknee=Vknee,
        useHeatPort=useHeatPort) annotation (Placement(visible=true, transformation(
            origin={10,-50},
            extent={{-10,-10},{10,10}},
            rotation=90)));
      Modelica.Electrical.Analog.Ideal.IdealThyristor idealThyristorN2(
        Ron=Ron,
        Goff=Goff,
        Vknee=Vknee,
        useHeatPort=useHeatPort) annotation (Placement(visible=true, transformation(
            origin={40,-50},
            extent={{-10,-10},{10,10}},
            rotation=90)));
      Modelica.Electrical.Analog.Interfaces.PositivePin ac_p
        annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin ac_n
        annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
      Modelica.Electrical.Analog.Interfaces.PositivePin dc_p
        annotation (Placement(transformation(extent={{90,50},{110,70}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin dc_n
        annotation (Placement(transformation(extent={{92,-70},{112,-50}})));
      Modelica.Blocks.Interfaces.BooleanInput firePositive
        "Fire signal for positive potential semiconductors" annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={-40,120})));
      Modelica.Blocks.Interfaces.BooleanInput fireNegative
        "Fire signal for negative potential semiconductors" annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={40,120})));
    equation
      if not useHeatPort then
        LossPower = idealThyristorP1.LossPower + idealThyristorP2.LossPower + idealThyristorN1.LossPower + idealThyristorN2.LossPower;
      end if;
      connect(idealThyristorP2.n,idealThyristorP1. n) annotation (Line(
          points={{40,60},{10,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealThyristorN1.p,idealThyristorN2. p) annotation (Line(
          points={{10,-60},{40,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealThyristorN2.n,idealThyristorP2. p) annotation (Line(
          points={{40,-40},{40,40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealThyristorP1.p,idealThyristorN1. n) annotation (Line(
          points={{10,40},{10,-40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealThyristorP1.n, dc_p) annotation (Line(
          points={{10,60},{100,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealThyristorN1.p, dc_n) annotation (Line(
          points={{10,-60},{102,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealThyristorN1.heatPort, heatPort) annotation (Line(
          points={{20,-50},{20,-100},{4.44089e-16,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(idealThyristorN2.heatPort, heatPort) annotation (Line(
          points={{50,-50},{50,-100},{4.44089e-16,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(idealThyristorP1.heatPort, heatPort) annotation (Line(
          points={{0,50},{-50,50},{-50,-100},{4.44089e-16,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(idealThyristorP2.heatPort, heatPort) annotation (Line(
          points={{30,50},{30,30},{-50,30},{-50,-100},{4.44089e-16,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(ac_p, idealThyristorP1.p) annotation (Line(
          points={{-100,60},{-80,60},{-80,20},{10,20},{10,40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(ac_n, idealThyristorN2.n) annotation (Line(
          points={{-100,-60},{-80,-60},{-80,-20},{40,-20},{40,-40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(firePositive, idealThyristorP1.fire) annotation (Line(
          points={{-40,120},{-40,80},{26,80},{26,57},{21,57}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(fireNegative, idealThyristorP2.fire) annotation (Line(
          points={{40,120},{40,80},{60,80},{60,56},{56,56},{56,57},{51,57}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(firePositive, idealThyristorN2.fire) annotation (Line(
          points={{-40,120},{-40,80},{26,80},{26,-43},{29,-43}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(fireNegative, idealThyristorN1.fire) annotation (Line(
          points={{40,120},{40,80},{60,80},{60,-10},{-10,-10},{-10,-43},{-1,-43}},
          color={255,0,255},
          smooth=Smooth.None));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics), Icon(coordinateSystem(
              preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
            graphics={
            Rectangle(
              extent={{-100,100},{100,-100}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Line(
              points={{-100,-100},{100,100}},
              color={0,0,127},
              smooth=Smooth.None),
            Text(
              extent={{-100,70},{0,50}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="AC"),
            Text(
              extent={{0,-50},{100,-70}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="DC"),             Text(
            extent={{-150,150},{150,110}},
            textString="%name",
            lineColor={0,0,255}),
            Text(
              extent={{-40,10},{40,-10}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="B2C")}));
    end FullBridge2Pulse;

    model DiodeBridge2mPulse "2*m pulse diode bridge rectifier"
      import Modelica.Constants.pi;
      parameter Integer m(final min=3)=3 "Number of phases";
      parameter Modelica.SIunits.Resistance Ron(final min=0) = 1.E-5
        "Closed thyristor resistance";
      parameter Modelica.SIunits.Conductance Goff(final min=0) = 1.E-5
        "Opened thyristor conductance";
      parameter Modelica.SIunits.Voltage Vknee(final min=0) = 0
        "Forward threshold voltage";
      extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(
         final T=293.15);
      Modelica.Electrical.MultiPhase.Ideal.IdealDiode idealDiodeP(
        final m=m,
        final Ron=fill(Ron, m),
        final Goff=fill(Goff, m),
        final Vknee=fill(Vknee, m),
        each final useHeatPort=useHeatPort) annotation (Placement(visible=true,
            transformation(
            origin={10,40},
            extent={{-10,-10},{10,10}},
            rotation=90)));
      Modelica.Electrical.MultiPhase.Ideal.IdealDiode idealDiodeN(
        final m=m,
        final Ron=fill(Ron, m),
        final Goff=fill(Goff, m),
        final Vknee=fill(Vknee, m),
        each final useHeatPort=useHeatPort) annotation (Placement(visible=true,
            transformation(
            origin={10,-40},
            extent={{-10,-10},{10,10}},
            rotation=90)));
      Modelica.Electrical.Analog.Interfaces.PositivePin dc_p
        annotation (Placement(transformation(extent={{90,50},{110,70}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin dc_n
        annotation (Placement(transformation(extent={{92,-70},{112,-50}})));
      Modelica.Electrical.MultiPhase.Interfaces.PositivePlug ac(final m=m)
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
      Modelica.Electrical.MultiPhase.Basic.Star starP(final m=m)
        annotation (Placement(transformation(extent={{20,50},{40,70}})));
      Modelica.Electrical.MultiPhase.Basic.Star starN(final m=m)
        annotation (Placement(transformation(extent={{20,-70},{40,-50}})));
      Modelica.Thermal.HeatTransfer.Components.ThermalCollector thermalCollector(final m=m) if
                                                                                    useHeatPort
        annotation (Placement(transformation(extent={{50,-100},{70,-80}})));
    equation
      if not useHeatPort then
        LossPower = sum(idealDiodeP.idealDiode.LossPower) + sum(idealDiodeN.idealDiode.LossPower);
      end if;
      connect(ac, idealDiodeP.plug_p) annotation (Line(
          points={{-100,4.44089e-16},{-100,0},{10,0},{10,30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealDiodeP.plug_p, idealDiodeN.plug_n) annotation (Line(
          points={{10,30},{10,-30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealDiodeP.plug_n, starP.plug_p) annotation (Line(
          points={{10,50},{10,60},{20,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(starP.pin_n, dc_p) annotation (Line(
          points={{40,60},{100,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealDiodeN.plug_p, starN.plug_p) annotation (Line(
          points={{10,-50},{10,-60},{20,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(starN.pin_n, dc_n) annotation (Line(
          points={{40,-60},{102,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(heatPort, thermalCollector.port_b) annotation (Line(
          points={{4.44089e-16,-100},{60,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(thermalCollector.port_a, idealDiodeN.heatPort) annotation (Line(
          points={{60,-80},{60,-40},{20,-40}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(idealDiodeP.heatPort, thermalCollector.port_a) annotation (Line(
          points={{20,40},{60,40},{60,-80}},
          color={191,0,0},
          smooth=Smooth.None));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics), Icon(coordinateSystem(
              preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
            graphics={
            Rectangle(
              extent={{-100,100},{100,-100}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Line(
              points={{-100,-100},{100,100}},
              color={0,0,127},
              smooth=Smooth.None),
            Text(
              extent={{-100,70},{0,50}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="AC"),
            Text(
              extent={{0,-50},{100,-70}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="DC"),             Text(
            extent={{-150,150},{150,110}},
            textString="%name",
            lineColor={0,0,255}),
            Text(
              extent={{-90,10},{90,-10}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="B2*%m%C")}));
    end DiodeBridge2mPulse;

    model HalfBridge2mPulse "2*m pulse half bridge SCR"
      import Modelica.Constants.pi;
      parameter Integer m(final min=3)=3 "Number of phases";
      parameter Modelica.SIunits.Resistance Ron(final min=0) = 1.E-5
        "Closed thyristor resistance";
      parameter Modelica.SIunits.Conductance Goff(final min=0) = 1.E-5
        "Opened thyristor conductance";
      parameter Modelica.SIunits.Voltage Vknee(final min=0) = 0
        "Forward threshold voltage";
      extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(
         final T=293.15);
      Modelica.Electrical.MultiPhase.Ideal.IdealThyristor idealThyristorP(
        final m=m,
        final Ron=fill(Ron,m),
        final Goff=fill(Goff,m),
        final Vknee=fill(Vknee,m),
        each final useHeatPort=useHeatPort)
           annotation (Placement(visible=true, transformation(
            origin={10,40},
            extent={{-10,-10},{10,10}},
            rotation=90)));
      Modelica.Electrical.MultiPhase.Ideal.IdealDiode idealDiodeN(
        final m=m,
        final Ron=fill(Ron, m),
        final Goff=fill(Goff, m),
        final Vknee=fill(Vknee, m),
        each final useHeatPort=useHeatPort)
        annotation (Placement(visible=true, transformation(
            origin={10,-40},
            extent={{-10,-10},{10,10}},
            rotation=90)));
      Modelica.Electrical.Analog.Interfaces.PositivePin dc_p
        annotation (Placement(transformation(extent={{90,50},{110,70}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin dc_n
        annotation (Placement(transformation(extent={{92,-70},{112,-50}})));
      Modelica.Electrical.MultiPhase.Interfaces.PositivePlug ac(final m=m)
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
      Modelica.Electrical.MultiPhase.Basic.Star starP(final m=m)
        annotation (Placement(transformation(extent={{20,50},{40,70}})));
      Modelica.Electrical.MultiPhase.Basic.Star starN(final m=m)
        annotation (Placement(transformation(extent={{20,-70},{40,-50}})));
      Modelica.Thermal.HeatTransfer.Components.ThermalCollector thermalCollector(final m=m) if
                                                                                    useHeatPort
        annotation (Placement(transformation(extent={{50,-100},{70,-80}})));
      Modelica.Blocks.Interfaces.BooleanInput firePositive[m]
        "Fire signals for positive potential semiconductors" annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={-40,120})));
    equation
      if not useHeatPort then
        LossPower = sum(idealThyristorP.idealThyristor.LossPower) + sum(idealDiodeN.idealDiode.LossPower);
      end if;
      connect(ac,idealThyristorP. plug_p) annotation (Line(
          points={{-100,4.44089e-16},{10,4.44089e-16},{10,30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealThyristorP.plug_p, idealDiodeN.plug_n)     annotation (Line(
          points={{10,30},{10,-30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealThyristorP.plug_n, starP.plug_p) annotation (Line(
          points={{10,50},{10,60},{20,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(starP.pin_n, dc_p) annotation (Line(
          points={{40,60},{100,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealDiodeN.plug_p, starN.plug_p)     annotation (Line(
          points={{10,-50},{10,-60},{20,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(starN.pin_n, dc_n) annotation (Line(
          points={{40,-60},{102,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(heatPort, thermalCollector.port_b) annotation (Line(
          points={{4.44089e-16,-100},{60,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(thermalCollector.port_a, idealDiodeN.heatPort)     annotation (Line(
          points={{60,-80},{60,-40},{20,-40}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(idealThyristorP.heatPort, thermalCollector.port_a) annotation (Line(
          points={{20,40},{60,40},{60,-80}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(firePositive, idealThyristorP.fire) annotation (Line(
          points={{-40,120},{-40,47},{-1,47}},
          color={255,0,255},
          smooth=Smooth.None));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics), Icon(coordinateSystem(
              preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
            graphics={
            Rectangle(
              extent={{-100,100},{100,-100}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Line(
              points={{-100,-100},{100,100}},
              color={0,0,127},
              smooth=Smooth.None),
            Text(
              extent={{-100,70},{0,50}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="AC"),
            Text(
              extent={{0,-50},{100,-70}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="DC"),             Text(
            extent={{-150,150},{150,110}},
            textString="%name",
            lineColor={0,0,255}),
            Text(
              extent={{-90,10},{90,-10}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="B2*%m%C")}));
    end HalfBridge2mPulse;

    model FullBridge2mPulse "2*m pulse full bridge SCR"
      import Modelica.Constants.pi;
      parameter Integer m(final min=3)=3 "Number of phases";
      parameter Modelica.SIunits.Resistance Ron(final min=0) = 1.E-5
        "Closed thyristor resistance";
      parameter Modelica.SIunits.Conductance Goff(final min=0) = 1.E-5
        "Opened thyristor conductance";
      parameter Modelica.SIunits.Voltage Vknee(final min=0) = 0
        "Forward threshold voltage";
      extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(
         final T=293.15);
      Modelica.Electrical.MultiPhase.Ideal.IdealThyristor idealThyristorP(
        final m=m,
        final Ron=fill(Ron,m),
        final Goff=fill(Goff,m),
        final Vknee=fill(Vknee,m),
        each final useHeatPort=useHeatPort)
           annotation (Placement(visible=true, transformation(
            origin={10,40},
            extent={{-10,-10},{10,10}},
            rotation=90)));
      Modelica.Electrical.MultiPhase.Ideal.IdealThyristor idealThyristorN(
        final m=m,
        final Ron=fill(Ron, m),
        final Goff=fill(Goff, m),
        final Vknee=fill(Vknee, m),
        each final useHeatPort=useHeatPort)
        annotation (Placement(visible=true, transformation(
            origin={10,-40},
            extent={{-10,-10},{10,10}},
            rotation=90)));
      Modelica.Electrical.Analog.Interfaces.PositivePin dc_p
        annotation (Placement(transformation(extent={{90,50},{110,70}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin dc_n
        annotation (Placement(transformation(extent={{92,-70},{112,-50}})));
      Modelica.Electrical.MultiPhase.Interfaces.PositivePlug ac(final m=m)
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
      Modelica.Electrical.MultiPhase.Basic.Star starP(final m=m)
        annotation (Placement(transformation(extent={{20,50},{40,70}})));
      Modelica.Electrical.MultiPhase.Basic.Star starN(final m=m)
        annotation (Placement(transformation(extent={{20,-70},{40,-50}})));
      Modelica.Thermal.HeatTransfer.Components.ThermalCollector thermalCollector(final m=m) if
                                                                                    useHeatPort
        annotation (Placement(transformation(extent={{50,-100},{70,-80}})));
      Modelica.Blocks.Interfaces.BooleanInput firePositive[m]
        "Fire signals for positive potential semiconductors" annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={-40,120})));
      Modelica.Blocks.Interfaces.BooleanInput fireNegative[m]
        "Fire signasl for negative potential semiconductors" annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={40,120})));
    equation
      if not useHeatPort then
        LossPower = sum(idealThyristorP.idealThyristor.LossPower) + sum(idealThyristorN.idealThyristor.LossPower);
      end if;
      connect(ac,idealThyristorP. plug_p) annotation (Line(
          points={{-100,0},{10,0},{10,30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealThyristorP.plug_p, idealThyristorN.plug_n) annotation (Line(
          points={{10,30},{10,-30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealThyristorP.plug_n, starP.plug_p) annotation (Line(
          points={{10,50},{10,60},{20,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(starP.pin_n, dc_p) annotation (Line(
          points={{40,60},{100,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealThyristorN.plug_p, starN.plug_p) annotation (Line(
          points={{10,-50},{10,-60},{20,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(starN.pin_n, dc_n) annotation (Line(
          points={{40,-60},{102,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(heatPort, thermalCollector.port_b) annotation (Line(
          points={{4.44089e-16,-100},{60,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(thermalCollector.port_a, idealThyristorN.heatPort) annotation (Line(
          points={{60,-80},{60,-40},{20,-40}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(idealThyristorP.heatPort, thermalCollector.port_a) annotation (Line(
          points={{20,40},{60,40},{60,-80}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(firePositive, idealThyristorP.fire) annotation (Line(
          points={{-40,120},{-40,47},{-1,47}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(fireNegative, idealThyristorN.fire) annotation (Line(
          points={{40,120},{40,80},{-20,80},{-20,-33},{-1,-33}},
          color={255,0,255},
          smooth=Smooth.None));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics), Icon(coordinateSystem(
              preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
            graphics={
            Rectangle(
              extent={{-100,100},{100,-100}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Line(
              points={{-100,-100},{100,100}},
              color={0,0,127},
              smooth=Smooth.None),
            Text(
              extent={{-100,70},{0,50}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="AC"),
            Text(
              extent={{0,-50},{100,-70}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="DC"),             Text(
            extent={{-150,150},{150,110}},
            textString="%name",
            lineColor={0,0,255}),
            Text(
              extent={{-90,10},{90,-10}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="B2*%m%C")}));
    end FullBridge2mPulse;
  end ACDC;

  package Icons
    extends Modelica.Icons.Package;
    model ExampleTemplate

      annotation (Icon(graphics={
            Ellipse(
              extent={{-100,100},{100,-100}},
              lineColor={175,175,175},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{-36,-60},{-36,60},{64,0},{-36,-60}},
              lineColor={175,175,175},
              smooth=Smooth.None,
              fillColor={175,175,175},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-4,46},{14,-44}},
              lineColor={255,255,255},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid)}));
    end ExampleTemplate;
  end Icons;
  annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2}),
        graphics),                                                                                                    Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})),
    uses(Modelica(version="3.2.2")));
end Modelica_Electrical_PowerConverters;
