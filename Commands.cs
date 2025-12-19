using Sandbox.ModAPI.Ingame;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VRageMath;

namespace IngameScript
{
    partial class Program
    {
        private Dictionary<string, Action<CommandLine>> commands;

        private void InitCommands()
        {
            commands = new Dictionary<string, Action<CommandLine>>
            {
                { "abort", cmd => AbortNav(false) },
                { "reload", CommandReload },
                { "maxthrustoverrideratio", CommandSetThrustRatio }, { "thrustratio", CommandSetThrustRatio },
                { "cruise", CommandCruise },
                { "retro", cmd => CommandRetrograde() }, { "retrograde", cmd => CommandRetrograde() },
                { "retroburn", cmd => CommandRetroburn() },
                { "prograde", cmd => CommandPrograde() },
                { "match", cmd => CommandSpeedMatch() }, { "speedmatch", cmd => CommandSpeedMatch() },
                { "orient", CommandOrient },
                { "calibrateturn", cmd => CommandCalibrateTurnTime() },
                { "thrust", CommandApplyThrust },
                { "journey", CommandJourney },
                { "autopilot", CommandAutopilot },
            };
        }

        private void HandleArgs(string argument)
        {
            if (String.IsNullOrWhiteSpace(argument))
            {
                return;
            }

            CommandLine cmd = new CommandLine(argument);

            foreach (var kv in commands)
            {
                if (cmd.Matches(0, kv.Key))
                {
                    kv.Value.Invoke(cmd);
                }
            }
        }

        private void CommandReload(CommandLine cmd)
        {
            AbortNav(false);
            LoadConfig(true);
        }

        private void CommandSetThrustRatio(CommandLine cmd)
        {
            if (cmd.Count < 2)
            {
                optionalInfo = "New override ratio argument not found!";
                return;
            }

            double result;
            if (!double.TryParse(cmd[1], out result))
            {
                optionalInfo = "Could not parse new override ratio";
                return;
            }

            if (result < 0 || result > 1.01)
            {
                optionalInfo = "Ratio must be between 0.0 and 1.0!";
                return;
            }

            result = MathHelper.Clamp(result, 0, 1);

            config.MaxThrustOverrideRatio = result;
            SaveConfig();

            if (cruiseController is SpeedMatch)
                thrustController.MaxThrustRatio = config.IgnoreMaxThrustForSpeedMatch ? 1f : (float)result;
            else
                thrustController.MaxThrustRatio = (float)result;

            optionalInfo = $"New thrust ratio set to {result:0.##}";
        }

        private void CommandCruise(CommandLine cmd)
        {
            if (cmd.Count < 3)
            {
                return;
            }

            AbortNav(false);
            optionalInfo = "";

            try
            {
                double desiredSpeed;
                Vector3D target;
                if (TryParseCruiseCommands(cmd, out desiredSpeed, out target))
                {
                    InitRetroCruise(target, desiredSpeed);
                }
            }
            catch (Exception e)
            {
                optionalInfo = e.ToString();
            }
        }

        private void InitRetroCruise(Vector3D target, double speed, RetroCruiseControl.RetroCruiseStage stage = RetroCruiseControl.RetroCruiseStage.None, bool saveConfig = true)
        {
            NavMode = NavModeEnum.Cruise;
            thrustController.MaxThrustRatio = (float)config.MaxThrustOverrideRatio;
            cruiseController = new RetroCruiseControl(target, speed, aimController, controller, gyros, thrustController, this, stage)
            {
                decelStartMarginSeconds = config.Ship180TurnTimeSeconds * 1.5,
            };
            cruiseController.CruiseTerminated += CruiseTerminated;
            config.PersistStateData = $"{NavModeEnum.Cruise}|{speed}|{stage}";
            Storage = target.ToString();
            if (saveConfig)
            {
                SaveConfig();
            }
        }

        private void CommandRetrograde()
        {
            AbortNav(false);
            optionalInfo = "";
            NavMode = NavModeEnum.Retrograde;
            cruiseController = new Retrograde(aimController, controller, gyros);
            cruiseController.CruiseTerminated += CruiseTerminated;
            config.PersistStateData = $"{NavModeEnum.Retrograde}";
            SaveConfig();
        }

        private void CommandRetroburn()
        {
            AbortNav(false);
            optionalInfo = "";
            NavMode = NavModeEnum.Retroburn;
            thrustController.MaxThrustRatio = (float)config.MaxThrustOverrideRatio;
            cruiseController = new Retroburn(aimController, controller, gyros, thrustController);
            cruiseController.CruiseTerminated += CruiseTerminated;
            config.PersistStateData = $"{NavModeEnum.Retroburn}";
            SaveConfig();
        }

        private void CommandPrograde()
        {
            AbortNav(false);
            optionalInfo = "";
            NavMode = NavModeEnum.Prograde;
            cruiseController = new Prograde(aimController, controller, gyros);
            cruiseController.CruiseTerminated += CruiseTerminated;
            config.PersistStateData = $"{NavModeEnum.Prograde}";
            SaveConfig();
        }

        private void CommandSpeedMatch()
        {
            AbortNav(false);
            optionalInfo = "";
            if (!wcApiActive)
            {
                try { wcApiActive = wcApi.Activate(Me); }
                catch { wcApiActive = false; }
            }
            if (!wcApiActive)
                return;
            var target = wcApi.GetAiFocus(Me.CubeGrid.EntityId);
            if ((target?.EntityId ?? 0) == 0)
                return;
            InitSpeedMatch(target.Value.EntityId);
        }

        private void CommandOrient(CommandLine cmd)
        {
            AbortNav(false);
            optionalInfo = "";
            if (!cmd.Gps.HasValue)
            {
                optionalInfo = "Incorrect orient command params, no gps detected";
                return;
            }

            InitOrient(cmd.Gps.Value.Position);
            optionalInfo = "";
        }

        private void CommandCalibrateTurnTime()
        {
            AbortNav(false);
            optionalInfo = "";
            NavMode = NavModeEnum.CalibrateTurnTime;
            cruiseController = new CalibrateTurnTime(config, aimController, controller, gyros);
            cruiseController.CruiseTerminated += CruiseTerminated;
            config.PersistStateData = $"{NavModeEnum.CalibrateTurnTime}";
            SaveConfig();
        }

        private void CommandApplyThrust(CommandLine cmd)
        {
            AbortNav(false);
            optionalInfo = "";
            float ratio;
            if (cmd.Count >= 3 && cmd.Matches(1, "set") && float.TryParse(cmd[2], out ratio))
            {
                if (ratio < 0 || ratio > 1.01)
                {
                    optionalInfo = "Ratio must be between 0.0 and 1.0!";
                }
                else
                {
                    optionalInfo = $"Forward thrust override set to {ratio * 100:0.###}%";
                    foreach (IMyThrust thrust in thrusters[Direction.Forward])
                    {
                        thrust.ThrustOverridePercentage = ratio;
                    }
                }
            }
        }

        private void CommandJourney(CommandLine cmd)
        {
            optionalInfo = "";
            if (cmd.Count < 2)
                return;
            optionalInfo = "";
            string failReason;
            if (cmd.Matches(1, "load"))
                InitJourney();
            else if (cruiseController is Journey && !((Journey)cruiseController).HandleJourneyCommand(cmd, out failReason))
                optionalInfo = failReason;
        }

        private void CommandAutopilot(CommandLine cmd)
        {
            // autopilot <speed> <forward dist in meters>
            // autopilot <speed> <X:Y:Z>
            // autopilot <speed> <GPS>
            if (cmd.Count < 3)
            {
                return;
            }

            AbortNav(false);
            optionalInfo = "";

            try
            {
                double desiredSpeed;
                Vector3D target;
                if (TryParseCruiseCommands(cmd, out desiredSpeed, out target))
                {
                    InitAutopilot(target, desiredSpeed);
                }
            }
            catch (Exception e)
            {
                optionalInfo = e.ToString();
            }
        }

        private void InitAutopilot(Vector3D target, double speed, bool saveConfig = true)
        {
            NavMode = NavModeEnum.Autopilot;
            thrustController.MaxThrustRatio = (float)config.MaxThrustOverrideRatio;
            var instance = new Autopilot(controller, thrustController)
            {
                Target = target,
                MaxSpeed = (float)speed,
            };
            cruiseController = instance;
            cruiseController.CruiseTerminated += CruiseTerminated;
            config.PersistStateData = $"{NavMode}|{speed}";
            Storage = target.ToString();
            if (saveConfig)
            {
                SaveConfig();
            }
        }

        // works for any command where the args are:
        // <cmdName> <speed> <forwardDist>
        // <cmdName> <speed> <X:Y:Z>
        // <cmdName> <speed> <GPS>
        private bool TryParseCruiseCommands(CommandLine cmd, out double desiredSpeed, out Vector3D target)
        {
            target = Vector3D.Zero;
            desiredSpeed = double.Parse(cmd[1]);

            double result;
            bool distanceCruise;
            bool containsGps = cmd.Gps.HasValue;
            if (distanceCruise = double.TryParse(cmd[2], out result))
            {
                target = controller.GetPosition() + (controller.WorldMatrix.Forward * result);
            }
            else if (containsGps)
            {
                target = cmd.Gps.Value.Position;
            }
            else
            {
                try
                {
                    string[] coords = cmd[2].Split(':');

                    double x = double.Parse(coords[0]);
                    double y = double.Parse(coords[1]);
                    double z = double.Parse(coords[2]);

                    target = new Vector3D(x, y, z);
                }
                catch (Exception e)
                {
                    optionalInfo = "Error occurred while parsing coords";
                    return false;
                }
            }

            Vector3D offsetTarget = Vector3D.Zero;

            if (!distanceCruise)
            {
                if (config.CruiseOffsetDist > 0)
                {
                    if (config.CruiseOffsetSideDist == 0)
                    {
                        optionalInfo = "Side offset cannot be zero when using offset";
                        return false;
                    }
                    offsetTarget += (target - controller.GetPosition()).SafeNormalize() * -config.CruiseOffsetDist;
                }
                if (config.CruiseOffsetSideDist > 0)
                {
                    offsetTarget += Vector3D.CalculatePerpendicularVector(target - controller.GetPosition()) * config.CruiseOffsetSideDist;
                }
            }

            target += offsetTarget;
            return true;
        }

        private void InitOrient(Vector3D target)
        {
            NavMode = NavModeEnum.Orient;
            cruiseController = new Orient(aimController, controller, gyros, target);
            cruiseController.CruiseTerminated += CruiseTerminated;
            config.PersistStateData = $"{NavModeEnum.Orient}";
            Storage = target.ToString();
            SaveConfig();
        }

        private void InitSpeedMatch(long targetId)
        {
            NavMode = NavModeEnum.SpeedMatch;
            thrustController.MaxThrustRatio = config.IgnoreMaxThrustForSpeedMatch ? 1f : (float)config.MaxThrustOverrideRatio;
            cruiseController = new SpeedMatch(targetId, wcApi, controller, Me, thrustController);
            cruiseController.CruiseTerminated += CruiseTerminated;
            config.PersistStateData = $"{NavModeEnum.SpeedMatch}|{targetId}";
            SaveConfig();
        }

        private void InitJourney()
        {
            NavMode = NavModeEnum.Journey;
            thrustController.MaxThrustRatio = (float)config.MaxThrustOverrideRatio;
            cruiseController = new Journey(aimController, controller, gyros, config.Ship180TurnTimeSeconds * 1.5, thrustController, this);
            cruiseController.CruiseTerminated += CruiseTerminated;
        }
    }
}
