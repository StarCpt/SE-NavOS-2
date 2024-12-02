using Sandbox.ModAPI.Ingame;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VRageMath;

namespace IngameScript
{
    public class Journey : ICruiseController
    {
        public event CruiseTerminateEventDelegate CruiseTerminated = delegate { };

        public string Name => nameof(Journey);

        IMyShipController shipController;
        IAimController aimControl;
        IList<IMyGyro> gyros;
        double decelStartMarginSeconds;
        IVariableThrustController thrustControl;
        Program prog;

        private ICruiseController cruiseControl;
        public List<Waypoint> waypoints;
        private bool started = false;
        private int currentStep = 0;

        public Journey(
            IAimController aimControl,
            IMyShipController controller,
            IList<IMyGyro> gyros,
            double decelStartMarginSeconds,
            IVariableThrustController thrustControl,
            Program program)
        {
            this.aimControl = aimControl;
            this.shipController = controller;
            this.gyros = gyros;
            this.decelStartMarginSeconds = decelStartMarginSeconds;
            this.thrustControl = thrustControl;
            this.prog = program;

            waypoints = ParseJourneySetup();
        }

        private List<Waypoint> ParseJourneySetup()
        {
            List<Waypoint> waypoints = new List<Waypoint>();
            List<string> lines = prog.config.JourneySetup;
            foreach (var line in lines)
            {
                Waypoint waypoint;
                if (TryParseWaypoint(line, out waypoint))
                {
                    waypoints.Add(waypoint);
                }
            }
            return waypoints;
        }

        public void AppendStatus(StringBuilder strb)
        {
            strb.Append("\n-- Journey Status --\n");
            strb.Append(!started ? "Awaiting Start Command..." : ("Current Step:" + (currentStep + 1) + "\nWaypoints:"));
            for (int i = 0; i < waypoints.Count; i++)
            {
                var step = waypoints[i];
                strb.Append("\n\n#" + (i + 1) + ": " + step.Name)
                    .Append($"\nTarget: X:{step.Target.X:0.0} Y:{step.Target.Y:0.0} Z:{step.Target.Z:0.0}")
                    .Append("\nSpeed: " + step.DesiredSpeed)
                    .Append("StopAtWaypoint: " + step.StopAtWaypoint);
            }
            strb.Append('\n');
            if (started)
                cruiseControl.AppendStatus(strb);
        }

        public bool HandleJourneyCommand(CommandLine cmd, out string failReason)
        {
            failReason = "Unknown Journey Command!";

            if (cmd.Count < 2)
            {
                return false;
            }
            else if (started)
            {
                failReason = "Cannot run Journey commands once started";
                return false;
            }

            //all commands:
            //journey init - handled elsewhere
            //journey start

            if (cmd.Matches(1, "start"))
            {
                if (waypoints.Count == 0)
                {
                    failReason = "No waypoints set!";
                    return false;
                }

                InitStep(0);
                return true;
            }

            return false;
        }

        public void InitStep(int index, bool approach = false)
        {
            if (!waypoints.IsValidIndex(index))
            {
                Terminate("Waypoint index is out of range");
            }

            var step = waypoints[index];
            Vector3D targetOffset = Vector3D.Zero;

            if (index == waypoints.Count - 1)
            {
                if (!approach && prog.config.CruiseOffsetSideDist > 0)
                {
                    targetOffset += Vector3D.CalculatePerpendicularVector(step.Target - shipController.GetPosition()) * prog.config.CruiseOffsetSideDist;
                }
                if (!approach && prog.config.CruiseOffsetDist > 0)
                {
                    targetOffset += (step.Target - shipController.GetPosition()).SafeNormalize() * -prog.config.CruiseOffsetDist;
                }
            }
            started = true;
            currentStep = index;
            if (step.StopAtWaypoint || index == waypoints.Count - 1)
            {
                cruiseControl = new RetroCruiseControl(step.Target + targetOffset, step.DesiredSpeed, aimControl, shipController, gyros, thrustControl, prog, false)
                {
                    decelStartMarginSeconds = this.decelStartMarginSeconds,
                };
            }
            else
            {
                cruiseControl = new Program.OneWayCruise(step.Target + targetOffset, step.DesiredSpeed, aimControl, shipController, gyros, thrustControl);
            }
            cruiseControl.CruiseTerminated += OnCruiseTerminated;
            SavePersistantData();
        }

        private void SavePersistantData()
        {
            prog.config.PersistStateData = $"{NavModeEnum.Journey}|{currentStep}";
            prog.Me.CustomData = prog.config.ToString();
        }

        public void Run()
        {
            if (started)
            {
                cruiseControl.Run();
            }
        }

        private void OnCruiseTerminated(ICruiseController sender, string reason)
        {
            if (reason == "JourneyTerminated")
            {
                return;
            }
            if (reason == "No functional gyros found")
            {
                Terminate(reason);
            }
            else if (Vector3D.DistanceSquared(shipController.GetPosition(), waypoints[currentStep].Target) <= 100 * 100)
            {
                currentStep++;
                if (currentStep < waypoints.Count)
                    InitStep(currentStep);
                else
                    Terminate("Destination Reached");
            }
            else
            {
                Terminate($"Journey terminated unexpectedly." +
                    $"\nRetroCruise terminate reason: {reason}" +
                    $"\nStep: {currentStep + 1} of {waypoints.Count}" +
                    $"\nDistanceToCurrentTarget: {Vector3D.Distance(shipController.GetPosition(), waypoints[currentStep].Target):0.00}" +
                    $"\nCurrent Speed: {shipController.GetShipSpeed():0.00}");
            }
        }

        public void Abort() => Terminate("Aborted");

        public void Terminate(string reason)
        {
            cruiseControl?.Terminate("JourneyTerminated");
            CruiseTerminated.Invoke(this, reason);
        }

        //format: <speed> <stopAtWaypoint: true/false> <gps>
        public static bool TryParseWaypoint(string line, out Waypoint waypoint)
        {
            string[] args = line.Split(' ');
            if (args.Length < 3)
            {
                waypoint = default(Waypoint);
                return false;
            }
            double speed;
            bool stopAtWaypoint;
            GPS gps;
            if (double.TryParse(args[0], out speed) && bool.TryParse(args[1], out stopAtWaypoint) && GPS.TryParse(line, out gps))
            {
                waypoint = new Waypoint(gps.Name, speed, gps.Position, stopAtWaypoint);
                return true;
            }
            waypoint = default(Waypoint);
            return false;
        }

        public struct Waypoint
        {
            public string Name;
            public double DesiredSpeed;
            public Vector3D Target;
            public bool StopAtWaypoint;

            public Waypoint(string name, double desiredSpeed, Vector3D target, bool stopAtWaypoint)
            {
                Name = name;
                DesiredSpeed = desiredSpeed;
                Target = target;
                StopAtWaypoint = stopAtWaypoint;
            }
        }
    }
}
