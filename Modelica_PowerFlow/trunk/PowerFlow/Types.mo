within PowerFlow;
package Types
  type ReferenceAngle "Reference angle"
    extends Modelica.SIunits.Angle;

    function equalityConstraint
      input ReferenceAngle theta1[:];
      input ReferenceAngle theta2[:];
      output Real[0] residue "No constraints";
    algorithm
      for i in 1:size(theta1, 1) loop
        assert(abs(theta1[i] - theta2[i]) < Modelica.Constants.eps, "angles theta1 and theta2 not equal over connection!");
      end for;
    end equalityConstraint;
  end ReferenceAngle;

  type PhaseDisplacement "Phase displacement between voltage and current"
    extends Modelica.SIunits.LossAngle;

    function equalityConstraint
      input PhaseDisplacement phi1;
      input PhaseDisplacement phi2;
      output Real[0] residue "No constraints";
    algorithm
      assert(abs(phi1 - phi2) < Modelica.Constants.eps, "phase displacements phi1 and phi2 not equal over connection!");
    end equalityConstraint;
  end PhaseDisplacement;

  package ComplexNumbers
    record Complex "complex number"
      Real re(start = 0) "real part";
      Real im(start = 0) "imaginary part";
    end Complex;

    function plus
      input Complex x1;
      input Complex x2;
      output Complex y(re = x1.re + x2.re, im = x1.im + x2.im);
    algorithm
    end plus;

    function minus
      input Complex x1;
      input Complex x2;
      output Complex y(re = x1.re - x2.re, im = x1.im - x2.im);
    algorithm
    end minus;

    function times
      input Complex x1;
      input Complex x2;
      output Complex y(re = x1.re*x2.re - x1.im*x2.im, im = x1.im*x2.re + x1.re*x2.im);
    algorithm
    end times;

    function divide
      input Complex x1;
      input Complex x2;
      output Complex y(re = (x1.re*x2.re + x1.im*x2.im)/(x2.re^2 + x2.im^2), im = (x1.im*x2.re - x1.re*x2.im)/(x2.re^2 + x2.im^2));
    algorithm
    end divide;

  end ComplexNumbers;

  record ComplexPower = ComplexNumbers.Complex (re(unit="W"), im(unit="var"));
  record ComplexResistance = ComplexNumbers.Complex (re(unit="Ohm"), im(unit=
            "Ohm"));
  record ComplexVoltage = ComplexNumbers.Complex (re(unit="V"), im(unit="V"));
  record ComplexCurrent = ComplexNumbers.Complex (re(unit="A"), im(unit="A"));
end Types;
