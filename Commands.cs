using Sandbox.ModAPI.Ingame;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using VRage.Game;
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
                { "radialin", _ => CommandRadialIn() },
                { "radialout", _ => CommandRadialOut() },
                { "match", cmd => CommandSpeedMatch() }, { "speedmatch", cmd => CommandSpeedMatch() },
                { "orient", CommandOrient },
                { "calibrateturn", cmd => CommandCalibrateTurnTime() },
                { "thrust", CommandApplyThrust },
                { "journey", CommandJourney },
                { "autopilot", CommandAutopilot },
                { "approach", CommandApproach },
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
            optionalInfo = "Config reloaded";
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

            if (result < 0 || result > 1)
            {
                optionalInfo = "Ratio must be between 0.0 and 1.0!";
                return;
            }

            result = MathHelper.Clamp(result, 0, 1);

            config.MaxThrustOverrideRatio = result;
            SaveConfig();

            if (CruiseController is SpeedMatch)
                thrustController.MaxForwardThrustRatio = config.IgnoreMaxThrustForSpeedMatch ? 1f : (float)result;
            else
                thrustController.MaxForwardThrustRatio = (float)result;

            optionalInfo = $"New thrust ratio set to {result:0.##}";
        }

        private void CommandCruise(CommandLine cmd)
        {
            AbortNav(false);
            optionalInfo = "";

            if (cmd.Count < 3)
            {
                optionalInfo = "Cruise arguments missing";
                return;
            }

            double desiredSpeed;
            Vector3D target;
            string error;
            if (TryParseCruiseCommands(cmd, out desiredSpeed, out target, out error))
            {
                InitRetroCruise(target, desiredSpeed);
            }
            else
            {
                optionalInfo = error;
            }
        }

        private void InitRetroCruise(Vector3D target, double speed, RetroCruiseControl.CruiseStage stage = RetroCruiseControl.CruiseStage.None, bool saveConfig = true)
        {
            speed = Math.Min(speed, this.GetWorldMaxSpeed());
            thrustController.MaxForwardThrustRatio = (float)config.MaxThrustOverrideRatio;
            NavMode = NavModeEnum.Cruise;
            CruiseController = new RetroCruiseControl(target, speed, aimController, controller, gyros, thrustController, this, stage)
            {
                ShipFlipTimeInSeconds = config.Ship180TurnTimeSeconds * 1.5,
            };
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
            CruiseController = new Retrograde(aimController, controller, gyros);
            config.PersistStateData = $"{NavModeEnum.Retrograde}";
            SaveConfig();
        }

        private void CommandRetroburn()
        {
            AbortNav(false);
            optionalInfo = "";
            thrustController.MaxForwardThrustRatio = (float)config.MaxThrustOverrideRatio;
            NavMode = NavModeEnum.Retroburn;
            CruiseController = new Retroburn(aimController, controller, gyros, thrustController);
            config.PersistStateData = $"{NavModeEnum.Retroburn}";
            SaveConfig();
        }

        private void CommandPrograde()
        {
            AbortNav(false);
            optionalInfo = "";
            NavMode = NavModeEnum.Prograde;
            CruiseController = new Prograde(aimController, controller, gyros);
            config.PersistStateData = $"{NavModeEnum.Prograde}";
            SaveConfig();
        }

        private void CommandRadialIn()
        {
            AbortNav(false);
            optionalInfo = "";
            NavMode = NavModeEnum.RadialIn;
            CruiseController = new RadialIn(aimController, controller, gyros);
            config.PersistStateData = NavModeEnum.RadialIn.ToString();
            SaveConfig();
        }

        private void CommandRadialOut()
        {
            AbortNav(false);
            optionalInfo = "";
            NavMode = NavModeEnum.RadialOut;
            CruiseController = new RadialOut(aimController, controller, gyros);
            config.PersistStateData = NavModeEnum.RadialOut.ToString();
            SaveConfig();
        }

        private void CommandSpeedMatch()
        {
            AbortNav(false);
            optionalInfo = "";
            if (!wcApiActive)
            {
                // try to activate the api again
                try { wcApiActive = wcApi.Activate(Me); }
                catch { wcApiActive = false; }
            }
            if (!wcApiActive)
            {
                optionalInfo = "WeaponCore API error";
                return;
            }
            var target = wcApi.GetAiFocus(Me.CubeGrid.EntityId);
            if ((target?.EntityId ?? 0) == 0)
            {
                optionalInfo = "Locked target not found";
                return;
            }
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
            CruiseController = new CalibrateTurnTime(config, aimController, controller, gyros);
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
            else if (CruiseController is Journey && !((Journey)CruiseController).HandleJourneyCommand(cmd, out failReason))
                optionalInfo = failReason;
        }

        private void CommandAutopilot(CommandLine cmd)
        {
            // autopilot <speed> <forward dist in meters>
            // autopilot <speed> <X:Y:Z>
            // autopilot <speed> <GPS>
            if (cmd.Count < 3)
            {
                optionalInfo = "Autopilot arguments missing";
                return;
            }

            AbortNav(false);
            optionalInfo = "";

            double desiredSpeed;
            Vector3D target;
            string error;
            if (TryParseCruiseCommands(cmd, out desiredSpeed, out target, out error))
            {
                InitAutopilot(target, desiredSpeed);
            }
            else
            {
                optionalInfo = error;
            }
        }

        private void CommandApproach(CommandLine cmd)
        {
            if (cmd.Count < 2)
            {
                optionalInfo = "Approach speed argument not found";
                return;
            }

            double approachSpeed;
            if (!double.TryParse(cmd[1], out approachSpeed))
            {
                optionalInfo = "Could not parse speed argument";
                return;
            }

            MyDetectedEntityInfo target = wcApi.GetAiFocus(Me.CubeGrid.EntityId) ?? default(MyDetectedEntityInfo);
            if (target.IsEmpty())
            {
                optionalInfo = "No locked target";
                return;
            }

            Vector3D targetPos = target.BoundingBox.Center;
            double targetRadius = BoundingSphereD.CreateFromBoundingBox(target.BoundingBox).Radius;

            // my radius + target radius
            double minRadius = Me.CubeGrid.WorldVolume.Radius + targetRadius;

            Vector3D myPos = Me.CubeGrid.WorldAABB.Center;

            if (Vector3D.DistanceSquared(targetPos, myPos) < (minRadius * minRadius))
            {
                optionalInfo = "Too close to target";
                return;
            }

            Vector3D targetDir = Vector3D.Normalize(targetPos - myPos);

            // perpendicular offset dir
            Vector3D offsetDir = Vector3D.CalculatePerpendicularVector(targetDir);

            // if the target is moving, attempt to avoid crashing into it by picking the opposite direction
            if (target.Velocity.LengthSquared() > 1 && Vector3D.Dot(offsetDir, target.Velocity) > 0)
            {
                offsetDir = -offsetDir;
            }

            Vector3D navTarget = targetPos + offsetDir * minRadius;

            // attempt to find closer intersection with minRadius sphere
            Vector3D navTargetDir = Vector3D.Normalize(navTarget - myPos);
            double? closestIntersection = new RayD(myPos, navTargetDir).Intersects(new BoundingSphereD(targetPos, minRadius));

            if (closestIntersection.HasValue)
            {
                navTarget = myPos + navTargetDir * closestIntersection.Value;
            }

            AbortNav(false);
            optionalInfo = "";

            optionalInfo = $"Approaching target {target.Name}";
            InitRetroCruise(navTarget, approachSpeed);
        }

        private void InitAutopilot(Vector3D target, double speed, bool saveConfig = true)
        {
            speed = Math.Min(speed, this.GetWorldMaxSpeed());
            thrustController.MaxForwardThrustRatio = (float)config.MaxThrustOverrideRatio;
            NavMode = NavModeEnum.Autopilot;
            CruiseController = new Autopilot(controller, thrustController)
            {
                Target = target,
                MaxSpeed = (float)speed,
            };
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
        private bool TryParseCruiseCommands(CommandLine cmd, out double desiredSpeed, out Vector3D target, out string error)
        {
            target = Vector3D.Zero;
            desiredSpeed = 0;

            try
            {
                if (!double.TryParse(cmd[1], out desiredSpeed))
                {
                    error = "Could not parse desired speed";
                    return false;
                }

                Vector3D controllerPos = controller.WorldAABB.Center;

                double result;
                bool distanceCruise;
                if (distanceCruise = double.TryParse(cmd[2], out result))
                {
                    target = controllerPos + (controller.WorldMatrix.Forward * result);
                }
                else if (cmd.Gps.HasValue)
                {
                    target = cmd.Gps.Value.Position;
                }
                else
                {
                    error = "Could not parse target";
                    return false;
                }

                bool useOffsets = true; // true by default
                if (cmd.Count >= 4 && cmd[3].ToLower() == "false")
                {
                    useOffsets = false;
                }

                useOffsets = useOffsets && !distanceCruise; // distance cruise doesn't use offsets

                if (useOffsets)
                {
                    Vector3D offsetTarget = Vector3D.Zero;

                    if (config.CruiseOffsetDist > 0)
                    {
                        if (config.CruiseOffsetSideDist == 0)
                        {
                            error = "Side offset cannot be zero when using offset";
                            return false;
                        }
                        offsetTarget += (target - controllerPos).SafeNormalize() * -config.CruiseOffsetDist;
                    }

                    if (config.CruiseOffsetSideDist > 0)
                    {
                        offsetTarget += Vector3D.CalculatePerpendicularVector(target - controllerPos) * config.CruiseOffsetSideDist;
                    }

                    target += offsetTarget;
                }

                error = null;
                return true;
            }
            catch (Exception e)
            {
                error = e.ToString();
                return false;
            }
        }

        private void InitOrient(Vector3D target)
        {
            NavMode = NavModeEnum.Orient;
            CruiseController = new Orient(aimController, controller, gyros, target);
            config.PersistStateData = $"{NavModeEnum.Orient}";
            Storage = target.ToString();
            SaveConfig();
        }

        private void InitSpeedMatch(long targetId)
        {
            thrustController.MaxForwardThrustRatio = config.IgnoreMaxThrustForSpeedMatch ? 1f : (float)config.MaxThrustOverrideRatio;
            NavMode = NavModeEnum.SpeedMatch;
            CruiseController = new SpeedMatch(targetId, wcApi, controller, Me, thrustController);
            config.PersistStateData = $"{NavModeEnum.SpeedMatch}|{targetId}";
            SaveConfig();
        }

        private void InitJourney()
        {
            thrustController.MaxForwardThrustRatio = (float)config.MaxThrustOverrideRatio;
            NavMode = NavModeEnum.Journey;
            CruiseController = new Journey(aimController, controller, gyros, config.Ship180TurnTimeSeconds * 1.5, thrustController, this);
        }
    }
}
