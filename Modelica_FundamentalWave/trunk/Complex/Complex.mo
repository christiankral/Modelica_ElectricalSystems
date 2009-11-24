package Complex "Definiton of complex data and operations" 
  extends Modelica.Icons.Library2;
  
  annotation (
      version="0.2.0",
      versionDate="2009-10-28",
      Documentation(info="<html>
<p>This library is intended to be integrated into the MSL anytime soon. Until then it serves as a common basis for various libraries, e.g. QuasiStationary, FundamentalWave.</p>
</html>", revisions="<html>
<table border=1>
 
<thead>
<tr><td>Version</td> <td>Date</td>  <td>Author(s)</td> <td>Comments</td></tr>
</thead>
 
<tbody>
<tr><td>0.1.0</td>  <td>2009-10-26</td>  <td>Christian Kral<br>Anton Haumer</td> <td></td> </tr>
</tbody>
 
</table>
 
</html>"),
    uses(Modelica(version="2.2.2")));
  
  record Complex "Record defining a complex number" 
    Real re "Real part of complex number";
    Real im "Imaginary part of complex number";
    annotation (Documentation(info=
                   "<html>
<p>
This record defines a complex number consisting of a real
and an imaginary part.
</html>"));
    
  end Complex;
  
connector ComplexSignal = Complex;
connector ComplexInput = input ComplexSignal 
  annotation (defaultComponentName="u",
  Coordsys(extent=[-100, -100; 100, 100],
    grid=[1,1],
    component=[20,20],
      scale=0.2),
  Icon(coordinateSystem(extent=[-100,-100; 100,100]),
       Polygon(points=[-100,100; 100,0; -100,-100; -100,100], style(
        color=45,
        rgbcolor={255,128,0},
        fillColor=45,
        rgbfillColor={255,128,0})),
    Text(
      extent=[100,101; 101,65],
      style(
        color=45,
        rgbcolor={255,128,0},
        fillColor=7,
        rgbfillColor={255,255,255},
        fillPattern=1),
      string="%name")),
  Diagram(Polygon(points=[0,50; 100,0; 0,-50; 0,50], style(
          color=45,
          rgbcolor={255,128,0},
          fillColor=45,
          rgbfillColor={255,128,0})),
      Text(
      extent=[-100,100; 100,60],
      string="%name",
        style(color=45, rgbcolor={255,128,0}))));
connector ComplexOutput = output ComplexSignal 
  annotation (defaultComponentName="y",
  Coordsys(extent=[-100, -100; 100, 100],
    grid=[1,1],
    component=[20,20]),
  Icon(Polygon(points=[-100,100; 100,0; -100,-100; -100,100], style(
          color=45,
          rgbcolor={255,128,0},
          fillColor=7,
          rgbfillColor={255,255,255}))),
  Diagram(Polygon(points=[-100,50; 0,0; -100,-50; -100,50], style(
          color=45,
          rgbcolor={255,128,0},
          fillColor=7,
          rgbfillColor={255,255,255})),
      Text(
      extent=[-100,100; 100,60],
      string="%name",
        style(color=45, rgbcolor={255,128,0}))));
  record ComplexVoltage = Complex (
      redeclare Modelica.SIunits.Voltage re,
      redeclare Modelica.SIunits.Voltage im) "Complex voltage";
  
  record ComplexCurrent = Complex (
      redeclare Modelica.SIunits.Current re,
      redeclare Modelica.SIunits.Current im) "Complex current";
  
  record ComplexImpedance=Complex (
      redeclare Modelica.SIunits.Resistance re,
      redeclare Modelica.SIunits.Reactance im) "Complex impedance";
  
  record ComplexAdmittance=Complex (
      redeclare Modelica.SIunits.Conductance re,
      redeclare Modelica.SIunits.Susceptance im) "Complex admittance";
  
  record ComplexPower=Complex (
      redeclare Modelica.SIunits.ActivePower re,
      redeclare Modelica.SIunits.ReactivePower im) "Complex power";
  record ComplexMagneticFlux = Complex (
    redeclare Modelica.SIunits.MagneticFlux re,
    redeclare Modelica.SIunits.MagneticFlux im) "Complex magnetic flux";
  record ComplexMagneticPotentialDifference = Complex (
    redeclare Modelica.SIunits.MagneticPotentialDifference re,
    redeclare Modelica.SIunits.MagneticPotentialDifference im) 
    "Complex magnetic potential difference";
  record ComplexReluctance = Complex (
    redeclare Modelica.SIunits.Reluctance re,
    redeclare Modelica.SIunits.Reluctance im) "Complex reluctance";
end Complex;
