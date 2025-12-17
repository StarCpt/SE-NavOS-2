// <mdk sortorder="-1" />
using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Text;
using VRage;
using VRage.Collections;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.GUI.TextPanel;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ObjectBuilders.Definitions;
using VRageMath;

namespace IngameScript
{
    public enum NavModeEnum //legacy: all future modes should be a class derived from ICruiseController
    {
        Sleep = -1,
        Idle = 0,
        Cruise = 1,
        Retrograde = 2,
        Prograde = 3,
        SpeedMatch = 4,
        Retroburn = 5,
        Orient = 6,
        CalibrateTurnTime = 7,
        Journey = 8,
        Autopilot = 9,
    }

    public enum Direction : byte
    {
        Forward,
        Backward,
        Left,
        Right,
        Up,
        Down
    }

    public partial class Program : MyGridProgram
    {
#region mdk preserve
//Config is in the CustomData

//lcd for logging
const string debugLcdName = "debugLcd";
const double throttleRt = 1;
const int printInterval = 10;

#endregion mdk preserve

        public NavModeEnum NavMode
        {
            get { return _navMode; }
            set
            {
                if (_navMode != value)
                {
                    var old = _navMode;
                    _navMode = value;
                    NavModeChanged(old, value);
                }
            }
        }
        private NavModeEnum _navMode = NavModeEnum.Idle;
        public bool IsNavIdle => NavMode == NavModeEnum.Idle;
        public bool IsNavSleep => NavMode == NavModeEnum.Sleep;

        private Dictionary<Direction, List<IMyThrust>> thrusters = new Dictionary<Direction, List<IMyThrust>>
        {
            { Direction.Forward, new List<IMyThrust>() },
            { Direction.Backward, new List<IMyThrust>() },
            { Direction.Right, new List<IMyThrust>() },
            { Direction.Left, new List<IMyThrust>() },
            { Direction.Up, new List<IMyThrust>() },
            { Direction.Down, new List<IMyThrust>() },
        };
        private List<IMyGyro> gyros = new List<IMyGyro>();
        private IMyShipController controller;

        private static StringBuilder debug;
        private IMyTextSurface debugLcd;
        private IMyTextSurface consoleLcd;
        private int counter = -1;
        private int idleCounter = 0;

        private IAimController aimController;
        private Profiler profiler;
        private WcPbApi wcApi;
        private bool wcApiActive = false;
        private ICruiseController cruiseController;
        private VariableThrustController thrustController;

        private DateTime bootTime;
        public const string programName = "NavOS";
        public const string versionStr = "2.15-dev";

        public Config config;

        public Program()
        {
            InitCommands();
            LoadConfig(false);
            UpdateBlocks();

            Runtime.UpdateFrequency = UpdateFrequency.Update1;
            bootTime = DateTime.UtcNow;

            aimController = new JitAim(Me.CubeGrid.GridSizeEnum);
            profiler = new Profiler(this);
            wcApi = new WcPbApi();
            thrustController = new VariableThrustController(thrusters, controller);

            try { wcApiActive = wcApi.Activate(Me); }
            catch { wcApiActive = false; }

            thrustController.UpdateThrusts();
            //AbortNav(false);

            TryRestoreNavState();
        }

        private void TryRestoreNavState()
        {
            if (String.IsNullOrWhiteSpace(config.PersistStateData))
                return;

            string[] args = config.PersistStateData.Split('|');
            NavModeEnum mode;
            
            if (args.Length == 0 || !Enum.TryParse<NavModeEnum>(args[0], out mode) || mode == NavModeEnum.Idle)
                return;

            AbortNav(false);

            try
            {
                string stateStr = null;
                if (mode == NavModeEnum.Cruise && args.Length >= 2)
                {
                    double desiredSpeed;
                    Vector3D target;
                    RetroCruiseControl.RetroCruiseStage stage = RetroCruiseControl.RetroCruiseStage.None;
                    if (double.TryParse(args[1], out desiredSpeed) && Vector3D.TryParse(Storage, out target) && (args.Length < 3 || Enum.TryParse(args[2], out stage)))
                    {
                        InitRetroCruise(target, desiredSpeed, stage, false);
                        stateStr = mode + " " + desiredSpeed;
                    }
                    else
                        stateStr = null;
                }
                if (mode == NavModeEnum.SpeedMatch && args.Length >= 2)
                {
                    long targetId;
                    if (long.TryParse(args[1], out targetId))
                    {
                        InitSpeedMatch(targetId);
                        stateStr = mode + " " + targetId;
                    }
                    else
                        stateStr = null;
                }
                else if (mode == NavModeEnum.Retrograde)
                {
                    CommandRetrograde();
                    stateStr = mode.ToString();
                }
                else if (mode == NavModeEnum.Retroburn)
                {
                    CommandRetroburn();
                    stateStr = mode.ToString();
                }
                else if (mode == NavModeEnum.Prograde)
                {
                    CommandPrograde();
                    stateStr = mode.ToString();
                }
                else if (mode == NavModeEnum.Orient)
                {
                    Vector3D target;
                    if (Vector3D.TryParse(Storage, out target))
                    {
                        InitOrient(target);
                        stateStr = mode.ToString();
                    }
                    else
                        stateStr = null;
                }
                else if (mode == NavModeEnum.Journey && args.Length >= 2)
                {
                    int step;
                    if (int.TryParse(args[1], out step))
                    {
                        NavMode = NavModeEnum.Journey;
                        thrustController.MaxThrustRatio = (float)config.MaxThrustOverrideRatio;
                        cruiseController = new Journey(aimController, controller, gyros, config.Ship180TurnTimeSeconds * 1.5, thrustController, this);
                        cruiseController.CruiseTerminated += CruiseTerminated;
                        ((Journey)cruiseController).InitStep(step);
                        stateStr = mode.ToString();
                    }
                }
                else if (mode == NavModeEnum.Autopilot && args.Length >= 2)
                {
                    double desiredSpeed;
                    Vector3D target;
                    if (double.TryParse(args[1], out desiredSpeed) && Vector3D.TryParse(Storage, out target))
                    {
                        InitAutopilot(target, desiredSpeed, false);
                        stateStr = mode + " " + desiredSpeed;
                    }
                    else
                    {
                        stateStr = null;
                    }
                }

                if (stateStr == null)
                    optionalInfo = $"Failed to restore {mode}";
                else
                    optionalInfo = $"Restored State: {stateStr}";
            }
            catch (Exception e)
            {
                config.PersistStateData = "";
                SaveConfig(false);
                optionalInfo = e.ToString();
            }
        }

        private void SaveConfig(bool updateblocks = true)
        {
            Me.CustomData = config.ToString();
            if (updateblocks)
            {
                UpdateBlocks();
            }
        }

        private void LoadConfig(bool updateBlocks)
        {
            if (!Config.TryParse(Me.CustomData, out config))
            {
                config = Config.Default;
            }
            SaveConfig(updateBlocks);
        }

        public void Main(string argument, UpdateType updateSource)
        {
            profiler.Run();
            counter++;

            if (argument.Length > 0)
            {
                HandleArgs(argument);
            }

            debugLcd?.WriteText(debug.ToString());

            if (IsNavIdle)
            {
                idleCounter++;
            }
            else if (cruiseController != null)
            {
                cruiseController.Run();
            }

            if (idleCounter >= 600)
            {
                NavMode = NavModeEnum.Sleep;
            }

            if (IsNavSleep || counter % (profiler.RunningAverageMs > throttleRt ? 60 : printInterval) == 0)
            {
                WritePbOutput();
            }
        }

        private void AbortNav(bool saveconfig = true)
        {
            cruiseController?.Abort();

            DisableThrustOverrides();
            DisableGyroOverrides();

            cruiseController = null;

            if (saveconfig)
            {
                config.PersistStateData = "";
                SaveConfig();
            }
        }

        private void CruiseTerminated(ICruiseController source, string reason)
        {
            optionalInfo = $"{source.Name} Terminated.\nReason: {reason}";

            cruiseController = null;

            LoadConfig(false);
            config.PersistStateData = "";
            SaveConfig();

            NavMode = NavModeEnum.Idle;
        }

        private void NavModeChanged(NavModeEnum old, NavModeEnum now)
        {
            idleCounter = 0;

            if (IsNavSleep)
            {
                Runtime.UpdateFrequency = UpdateFrequency.None;
                optionalInfo = "Sleeping...";
            }
            else if (old == NavModeEnum.Sleep)
            {
                Runtime.UpdateFrequency = UpdateFrequency.Update1;
                optionalInfo = "";
            }
        }

        private void DisableThrustOverrides()
        {
            foreach (var list in thrusters.Values)
                for (int i = 0; i < list.Count; i++)
                    list[i].ThrustOverridePercentage = 0;
        }

        private void DisableGyroOverrides()
        {
            foreach (var gyro in gyros)
            {
                gyro.GyroOverride = false;
                gyro.Pitch = 0;
                gyro.Yaw = 0;
                gyro.Roll = 0;
            }
        }

        private void UpdateBlocks()
        {
            foreach (var list in thrusters.Values)
            {
                list.Clear();
            }

            var blocks = new List<IMyTerminalBlock>();
            GridTerminalSystem.GetBlocksOfType(blocks, i => i.CubeGrid == Me.CubeGrid);

            var controllers = blocks.OfType<IMyShipController>().Where(b => b.CustomName.Contains(config.ShipControllerTag)).ToList();
            if (controllers.Count == 0)
                throw new Exception($"No cockpit with \"{config.ShipControllerTag}\" found!");
            else controller = controllers[0];


            var tempThrusters = new List<IMyThrust>();
            GridTerminalSystem.GetBlockGroupWithName(config.ThrustGroupName)?.GetBlocksOfType(tempThrusters, i => i.CubeGrid == Me.CubeGrid);

            if (tempThrusters.Count == 0)
                GridTerminalSystem.GetBlocksOfType(tempThrusters, i => i.CubeGrid == Me.CubeGrid);

            if (tempThrusters.Count == 0)
                throw new Exception("bruh, this ship's got no thrusters!!");

            foreach (var thruster in tempThrusters)
            {
                switch (GetBlockDirection(thruster.WorldMatrix.Forward, controller.WorldMatrix))
                {
                    case Direction.Backward: thrusters[Direction.Forward].Add(thruster); break;
                    case Direction.Forward: thrusters[Direction.Backward].Add(thruster); break;
                    case Direction.Left: thrusters[Direction.Right].Add(thruster); break;
                    case Direction.Right: thrusters[Direction.Left].Add(thruster); break;
                    case Direction.Down: thrusters[Direction.Up].Add(thruster); break;
                    case Direction.Up: thrusters[Direction.Down].Add(thruster); break;
                }
            }

            GridTerminalSystem.GetBlockGroupWithName(config.GyroGroupName)?.GetBlocksOfType(gyros, i => i.CubeGrid == Me.CubeGrid && i.IsFunctional);

            if (gyros.Count == 0)
                GridTerminalSystem.GetBlocksOfType(gyros, i => i.CubeGrid == Me.CubeGrid && i.IsFunctional);

            if (gyros.Count == 0)
                throw new Exception("No gyros");

            debugLcd = TryGetBlockWithName<IMyTextSurfaceProvider>(debugLcdName)?.GetSurface(0);
            if (debugLcd != null)
                debug = new StringBuilder();
            consoleLcd = TryGetBlockWithName<IMyTextSurfaceProvider>(config.ConsoleLcdName)?.GetSurface(0);
        }

        private T TryGetBlockWithName<T>(string name) where T : class
        {
            IMyTerminalBlock block = GridTerminalSystem.GetBlockWithName(name);

            return block is T ? (T)block : default(T);
        }

        public static Direction GetBlockDirection(Vector3D vector, MatrixD refMatrix)
        {
            if (vector == refMatrix.Forward) return Direction.Forward;
            if (vector == refMatrix.Backward) return Direction.Backward;
            if (vector == refMatrix.Right) return Direction.Right;
            if (vector == refMatrix.Left) return Direction.Left;
            if (vector == refMatrix.Up) return Direction.Up;
            if (vector == refMatrix.Down) return Direction.Down;
            throw new Exception("Unknown direction");
        }

        private StringBuilder pbOut = new StringBuilder();
        public static string optionalInfo = "";

        private void WritePbOutput()
        {
            //PB Output
            const string programInfoStr = programName + " v" + versionStr + " | ";
            const string commandStr = @"
All Commands:
Cruise <Speed> <distance>
Cruise <Speed> <X:Y:Z>
Cruise <Speed> <GPS>
Retro/Retrograde
Prograde
Retroburn
Match
Orient <GPS>
Abort
ThrustRatio <ratio0to1>
Thrust Set <ratio>
CalibrateTurn
Journey Load
Journey Start
";
            string avgRtStr = profiler.RunningAverageMs.ToString("0.0000");

            pbOut.Append(programInfoStr).Append(avgRtStr);
            TimeSpan upTime = DateTime.UtcNow - bootTime;
            pbOut.Append("\nUptime: ").Append(SecondsToDuration(upTime.TotalSeconds));
            pbOut.Append("\nMode: ").AppendLine(NavMode.ToString());

            if (optionalInfo.Length > 0)
            {
                pbOut.AppendLine();
                pbOut.AppendLine(optionalInfo);
            }

            AppendNavInfo(pbOut);

            pbOut.Append("\n-- Loaded Config --\n" +
                nameof(config.MaxThrustOverrideRatio) + "=" + config.MaxThrustOverrideRatio.ToString() + "\n" +
                nameof(config.IgnoreMaxThrustForSpeedMatch) + "=" + config.IgnoreMaxThrustForSpeedMatch.ToString() + "\n" +
                nameof(config.ShipControllerTag) + "=" + config.ShipControllerTag + "\n" +
                nameof(config.ThrustGroupName) + "=" + config.ThrustGroupName + "\n" +
                nameof(config.GyroGroupName) + "=" + config.GyroGroupName + "\n" +
                nameof(config.ConsoleLcdName) + "=" + config.ConsoleLcdName + "\n" +
                nameof(config.CruiseOffsetDist) + "=" + config.CruiseOffsetDist.ToString() + "\n" +
                nameof(config.CruiseOffsetSideDist) + "=" + config.CruiseOffsetSideDist.ToString() + "\n" +
                nameof(config.Ship180TurnTimeSeconds) + "=" + config.Ship180TurnTimeSeconds.ToString() + "\n" +
                nameof(config.MaintainDesiredSpeed) + "=" + config.MaintainDesiredSpeed.ToString() + "\n");

            consoleLcd?.WriteText(pbOut);

            if (debugLcd != null)
                pbOut.Append("\nDebug: ").Append(debugLcd != null);

            pbOut.Append(commandStr)

            .Append("\n-- Detected Blocks --")
            .Append("\nConsoleLcd: " + (consoleLcd != null))
            .Append("\nDebugLcd: " + (debugLcd != null)).AppendLine()
            .Append(thrusters[Direction.Forward].Count + " Forward Thrusters\n")
            .Append(thrusters[Direction.Backward].Count + " Backward Thrusters\n")
            .Append(thrusters[Direction.Right].Count + " Right Thrusters\n")
            .Append(thrusters[Direction.Left].Count + " Left Thrusters\n")
            .Append(thrusters[Direction.Up].Count + " Up Thrusters\n")
            .Append(thrusters[Direction.Down].Count + " Down Thrusters\n")
            .Append(gyros.Count + " Gyros")
            .Append("\n\n-- Runtime Information --")
            .Append("\nLast: " + Runtime.LastRunTimeMs)
            .Append("\nAverage: " + avgRtStr)
            .Append("\nMax: " + profiler.MaxRuntimeMsFast);

            Echo(pbOut.ToString());

            pbOut.Clear();
        }

        private void AppendNavInfo(StringBuilder strb)
        {
            //placeholder - 
            cruiseController?.AppendStatus(strb);
        }

        public static string SecondsToDuration(double seconds, bool fractions = false)
        {
            if (double.IsNaN(seconds))
                return "NaN";

            if (double.IsInfinity(seconds))
                return "Infinity";

            seconds = Math.Abs(seconds);

            int hours = (int)seconds / 3600;

            seconds %= 3600;

            int minutes = (int)seconds / 60;

            seconds %= 60;

            if (hours > 0) return $"{hours.ToString("00")}:{minutes.ToString("00")}:{seconds.ToString("00")}{(fractions ? (seconds - (int)seconds).ToString(".000") : "")}";
            else return $"{minutes.ToString("00")}:{seconds.ToString("00")}";
        }

        public static void Log(string message) => debug?.AppendLine(message);

        public void SetStorage(string str) => Storage = str;
    }
}
