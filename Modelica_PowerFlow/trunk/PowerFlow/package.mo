within ;
package PowerFlow "Library for electrical power flow calculations"


  replaceable package PackagePhaseSystem =
      PowerFlow.PhaseSystems.ThreePhaseSymmetric "Default phase system"
    annotation (choicesAllMatching=true);


  annotation (version="0.3", uses(Modelica(version="3.1")));
end PowerFlow;
