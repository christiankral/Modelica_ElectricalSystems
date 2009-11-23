within PowerFlow.Examples;
model PowerWorld "Interoperation of wind power and thermal power"
  extends Modelica.Icons.Example;

  PowerFlow.Units.WindFarm windFarm(redeclare package PhaseSystem = 
        PowerFlow.PhaseSystems.DirectCurrent) 
                               annotation (Placement(transformation(extent={{-50,60},
            {-30,80}},           rotation=0)));
  PowerFlow.Units.City city 
                       annotation (                      Placement(
        transformation(extent={{60,-50},{80,-30}},
                                                rotation=0)));
  PowerFlow.Units.LoadDispatcher dispatcher 
    annotation (                           Placement(transformation(extent={{-90,-60},
            {-70,-40}}, rotation=0)));
  PowerFlow.Units.PowerPlant powerPlant(primaryControlMax=40) 
                                   annotation (
      Placement(transformation(extent={{-62,-10},{-40,12}},
                                                          rotation=0)));
  annotation (
    experiment(StopTime=86400),
    Commands(file(ensureSimulated=true)="plot summary.mos" "plot summary",
             file(ensureSimulated=true)="plot powerPlant.mos" "plot powerPlant",
             file(ensureSimulated=true)="plot hydroPlant.mos" "plot hydroPlant"),
    Documentation(info="<html>
<p>
This example models a control area for power distribution in island mode, i.e. without connection to a larger net.
It contains the following consumers and producers:
<ul>
<li>a city with a load of about 1000 MW, i.e. 1 Mio inhabitants,</li>
<li>a thermal power plant with
   <ul>
   <li>800 MW nominal power</li>
   <li>60 MW for secondary frequency control</li>
   <li>40 MW for primary frequency control</li>
   </ul></li>
<li>a wind park with a max power of 300 MW,</li>
<li>a hydro plant providing:
   <ul> 
   <li>50 MW base load using a river turbine</li>
   <li>+-25 MW pumping power to support the day/night load cycle</li>
   <li>up to 200 MW peak power on demand.</li>
   </ul></li>
</ul>
</p>
<p>
The following switches/features are provided:
<ul>
<li><b>powerPlant.Modakond</b>: enhance the frequency/power control of the power plant to reduce throttle losses by utilizing condensate stop (see powerPlant.hotwellLevel.y, powerPlant.throttleReserve.y, powerPlant.pressureLoss.y, powerPlant.throttleCosts.y)</li>
<li><b>windForm.cut_off</b>: suddenly take off the wind farm as the wind speed exceeds the cut-off speed, e.g. in case of a storm</li>
</ul>
</p>
    </html>"),
    Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
            {100,100}}), graphics));
  Components.VoltageConverter trafoPlant(ratio=10/380) 
    annotation (Placement(transformation(extent={{-36,-6},{-24,6}})));
  Components.VoltageConverter distribution(ratio=380/50) 
    annotation (Placement(transformation(extent={{44,-46},{56,-34}})));
  Components.Inverter HVDC 
    annotation (Placement(transformation(extent={{-16,34},{-4,46}})));
  PowerFlow.Units.HydroPlant hydroPlant(primaryControlMax=200) 
    annotation (Placement(transformation(extent={{80,20},{60,40}})));
  Components.VoltageConverter trafoHydro(ratio=380/10) 
    annotation (Placement(transformation(extent={{44,24},{56,36}})));
  Components.Impedance linePlant(               R=1, L=1/50) 
    annotation (Placement(transformation(extent={{-16,-46},{-4,-34}})));
  Components.Impedance lineWind(R=1, L=1/50) 
                                     annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=-90,
        origin={10,10})));
  Components.Impedance lineHydro(               R=1, L=1/50) 
                                      annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=-90,
        origin={40,-10})));
  Modelica.Blocks.Sources.RealExpression frequency(y=der(
        distribution.PhaseSystem.angle(distribution.terminal_p.theta)))
    "Frequency in the distribution net" annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-80,-20})));
equation
  connect(powerPlant.terminal, trafoPlant.terminal_p) 
                                                 annotation (Line(
      points={{-40,6.10623e-16},{-39,6.10623e-16},{-39,4.21885e-16},{-38,
          4.21885e-16},{-38,0},{-36,0}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(distribution.terminal_n, city.terminal) annotation (Line(
      points={{56,-40},{60,-40}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(windFarm.terminal, HVDC.terminal_dc) annotation (Line(
      points={{-30,70},{-20,70},{-20,40},{-16,40}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(dispatcher.plantDispatch, powerPlant.plantDispatch) annotation (Line(
      points={{-73,-46},{-70,-46},{-70,-6},{-62,-6}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(dispatcher.hydroDispatch, hydroPlant.hydroDispatch) annotation (Line(
      points={{-73,-52},{-70,-52},{-70,-80},{90,-80},{90,30},{80,30}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(trafoPlant.terminal_n, linePlant.terminal_p) annotation (Line(
      points={{-24,-1.88738e-16},{-20,-1.88738e-16},{-20,-40},{-16,-40}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(linePlant.terminal_n, distribution.terminal_p) annotation (Line(
      points={{-4,-40},{44,-40}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(HVDC.terminal, lineWind.terminal_p) annotation (Line(
      points={{-4,40},{10,40},{10,16}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(lineWind.terminal_n, distribution.terminal_p) annotation (Line(
      points={{10,4},{10,-38},{44,-38},{44,-40}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(lineHydro.terminal_n, distribution.terminal_p) annotation (Line(
      points={{40,-16},{40,-36},{44,-36},{44,-40}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(frequency.y, dispatcher.frequency) annotation (Line(
      points={{-80,-31},{-80,-43}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(hydroPlant.terminal, trafoHydro.terminal_n) annotation (Line(
      points={{60,30},{56,30}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(trafoHydro.terminal_p, lineHydro.terminal_p) annotation (Line(
      points={{44,30},{40,30},{40,-4}},
      color={0,0,0},
      smooth=Smooth.None));
end PowerWorld;
