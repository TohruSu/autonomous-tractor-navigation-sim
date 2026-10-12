namespace TractorNavigation;

public interface IPathController
{
    string Name { get; }

    double ComputeSteering(VehicleState state, Route route, double wheelbase);
}
