using System;
using System.Linq;
using UnityEngine;

namespace RemoteTech.FlightComputer.Commands
{
    public class ManeuverCommand : AbstractCommand
    {
        /// <summary>Index id of this maneuver node from patchedConicSolver.maneuverNodes list</summary>
        [Persistent] public int NodeIndex;
        /// <summary></summary>
        [Persistent] public string KaCItemId = String.Empty;

        public double OriginalDelta;
        public double RemainingTime;
        public double RemainingDelta;
        public ManeuverNode Node;
        public bool EngineActivated { get; private set; }
        public override int Priority { get { return 0; } }

        private double throttle = 1.0f;
        private double lowestDeltaV = 0.0;
        private bool abortOnNextExecute = false;

        double ACCELERATION_EPSILON = 0.000001;
        double KERBIN_GRAVITY = 9.81;
        double totalThrust;

        public override string Description
        {
            get
            {
                if (RemainingTime > 0 || RemainingDelta > 0)
                {
                    string flightInfo = "Executing maneuver: " + RemainingDelta.ToString("F2") +
                                        "m/s" + Environment.NewLine + "Remaining duration: ";

                    flightInfo += this.EngineActivated ? RTUtil.FormatDuration(RemainingTime) : "-:-";

                    return flightInfo + Environment.NewLine + base.Description;
                }
                else
                    return "Execute planned maneuver" + Environment.NewLine + base.Description;
            }
        }
        public override string ShortName { get { return "Execute maneuver node"; } }

        public override bool Pop(FlightComputer f)
        {
            var burn = f.ActiveCommands.FirstOrDefault(c => c is BurnCommand);
            if (burn != null) {
                f.Remove (burn);
            }

            OriginalDelta = Node.DeltaV.magnitude;
            RemainingDelta = this.getRemainingDeltaV(f);
            this.EngineActivated = true;

            ShipState vessel = new ShipState();
            Tally propellantsConsumed = new Tally();

            GetThrustInfo(propellantsConsumed, out totalThrust);
            double thrustToMass = totalThrust / f.Vessel.GetTotalMass();

            if (thrustToMass == 00)
            {
                this.EngineActivated = false;
            }
            else
            {
                RemainingTime = GetBurnTime(RemainingDelta);
            }

            /*
            double thrustToMass = FlightCore.GetTotalThrust(f.Vessel) / f.Vessel.GetTotalMass();
            if (thrustToMass == 0.0) {
                this.EngineActivated = false;
                RTUtil.ScreenMessage("[Flight Computer]: No engine to carry out the maneuver.");
            } else {
                RemainingTime = GetBurnTime(RemainingDelta);
            }*/

            return true;
        }

        // paste from BetterBurnTime.cs
        //

        double GetBurnTime(double dVremaining)
        {
            ShipState vessel = new ShipState();
            Tally propellantsConsumed = new Tally();

            // How thirsty are we?
            GetThrustInfo(propellantsConsumed, out totalThrust);
            if (totalThrust < ACCELERATION_EPSILON)
            {
                // Can't thrust, will take forever.
                return double.PositiveInfinity;
            }

            // If infinite fuel is turned on, or if the "use simple acceleration" config
            // option is set, just do a simple dV calculation.
            if (CheatOptions.InfinitePropellant)
            {
                return dVremaining * vessel.TotalMass / totalThrust;
            }

            // How long can we burn until we run out of fuel?
            double maxBurnTime = CalculateMaxBurnTime(vessel, propellantsConsumed);

            // How much fuel do we need to burn to get the dV we want?
            double totalConsumption = propellantsConsumed.Sum; // tons/second
            double exhaustVelocity = totalThrust / totalConsumption; // meters/second
            double massRatio = Math.Exp(dVremaining / exhaustVelocity);
            double currentTotalShipMass = vessel.TotalMass;
            double fuelMass = currentTotalShipMass * (1.0 - 1.0 / massRatio);
            double burnTimeNeeded = fuelMass / totalConsumption;
            if (burnTimeNeeded < maxBurnTime)
            {
                // We can burn that long! We're done.
                return burnTimeNeeded;
            }

            // Uh oh.  There's not enough fuel to get that much dV.  Here's what we'll do:
            // Take the amount of burn time that we can actually handle, and do that.
            // Then assume that we'll do constant acceleration at that speed for the
            // remainder of the dV.
            double fuelBurned = totalConsumption * maxBurnTime; // tons
            double emptyMass = currentTotalShipMass - fuelBurned;
            double realdV = (totalThrust / totalConsumption) * Math.Log(currentTotalShipMass / emptyMass);
            double highestAcceleration = totalThrust / emptyMass;
            double overflowdV = dVremaining - realdV;
            return maxBurnTime + overflowdV / highestAcceleration;
        }

        /// <summary>
        /// Get the vessel's acceleration ability, in m/s2
        /// </summary>
        /// <param name="propellantsConsumed">All the propellants consumed, in tons/second for each one</param>
        /// <param name="totalThrust">The total thrust produced, in kilonewtons</param>
        private void GetThrustInfo(Tally propellantsConsumed, out double totalThrust)
        {
            // Add up all the thrust for all the active engines on the vessel.
            // We do this as a vector because the engines might not be parallel to each other.
            Vector3 totalThrustVector = Vector3.zero;
            totalThrust = 0.0F;
            propellantsConsumed.Zero();
            int lastEngineCount = -1;
            ShipState vessel = new ShipState();

            Tally availableResources = vessel.AvailableResources;
            int engineCount = 0;
            for (int engineIndex = 0; engineIndex < vessel.ActiveEngines.Count; ++engineIndex)
            {
                ModuleEngines engine = vessel.ActiveEngines[engineIndex];
                if (engine.thrustPercentage > 0)
                {
                    double engineKilonewtons = engine.ThrustLimit();
                    if (!CheatOptions.InfinitePropellant)
                    {
                        // Possible future consideraiton:
                        // Get the vacuum Isp from engine.atmosphereCurve.Evaluate(0), rather than ask
                        // for engine.realIsp, because there may be mods that tinker with the atmosphere
                        // curve, which changes the actual Isp that the game uses for vacuum without
                        // updating engine.realIsp.
                        double engineIsp = engine.realIsp;

                        double engineTotalFuelConsumption = engineKilonewtons / (KERBIN_GRAVITY * engineIsp); // tons/sec
                        double ratioSum = 0.0;
                        bool isStarved = false;
                        for (int propellantIndex = 0; propellantIndex < engine.propellants.Count; ++propellantIndex)
                        {
                            Propellant propellant = engine.propellants[propellantIndex];
                            if (!ShouldIgnore(propellant.name))
                            {
                                if (!availableResources.Has(propellant.name))
                                {
                                    isStarved = true;
                                    break;
                                }
                                ratioSum += propellant.ratio;
                            }
                        }
                        if (isStarved) continue;
                        if (ratioSum > 0)
                        {
                            double ratio = 1.0 / ratioSum;
                            for (int propellantIndex = 0; propellantIndex < engine.propellants.Count; ++propellantIndex)
                            {
                                Propellant propellant = engine.propellants[propellantIndex];
                                if (!ShouldIgnore(propellant.name))
                                {
                                    double consumptionRate = ratio * propellant.ratio * engineTotalFuelConsumption; // tons/sec
                                    propellantsConsumed.Add(propellant.name, consumptionRate);
                                }
                            }
                        }
                    } // if we need to worry about fuel
                    ++engineCount;
                    totalThrustVector += engine.Forward() * (float)engineKilonewtons;
                } // if the engine is operational
            } // for each engine module on the part
            totalThrust = totalThrustVector.magnitude;
            if (engineCount != lastEngineCount)
            {
                lastEngineCount = engineCount;
            }
        }

        /// <summary>
        /// Calculate how long we can burn at full throttle until something important runs out.
        /// </summary>
        /// <param name="vessel"></param>
        /// <param name="propellantsConsumed"></param>
        /// <param name="propellantsAvailable"></param>
        /// <param name="maxBurnTime"></param>
        private static double CalculateMaxBurnTime(ShipState vessel, Tally propellantsConsumed)
        {
            double maxBurnTime = double.PositiveInfinity;
            Tally availableResources = vessel.AvailableResources;
            foreach (string resourceName in propellantsConsumed.Keys)
            {
                if (ShouldIgnore(resourceName))
                {
                    // ignore this for burn time, it's replenishable
                    continue;
                }
                if (!availableResources.Has(resourceName))
                {
                    // we're all out!
                    return 0.0;
                }
                double availableAmount = availableResources[resourceName];
                double rate = propellantsConsumed[resourceName];
                double burnTime = availableAmount / rate;
                if (burnTime < maxBurnTime) maxBurnTime = burnTime;
            }
            return maxBurnTime;
        }

        private static bool ShouldIgnore(string propellantName)
        {
            return "ElectricCharge".Equals(propellantName);
        }

        // end paste from BetterBurnTime.cs
        //


        /// <summary>
        /// Gets the current remaining delta velocity for this maneuver burn determined by the vessels burn vector of the passed FlightComputer instance.
        /// </summary>
        /// <param name="computer">FlightComputer instance to determine remaining delta velocity by.</param>
        /// <returns>Remaining delta velocity in m/s^2</returns>
        private double getRemainingDeltaV(FlightComputer computer)
        {
            return this.Node.GetBurnVector(computer.Vessel.orbit).magnitude;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="computer">FlightComputer instance of the computer of the vessel.</param>
        private void AbortManeuver(FlightComputer computer)
        {
            RTUtil.ScreenMessage("[Flight Computer]: Maneuver removed");
            if (computer.Vessel.patchedConicSolver != null)
            {
                Node.RemoveSelf();
            }
            // enqueue kill rot
            computer.Enqueue(AttitudeCommand.KillRot(), true, true, true);
        }

        /// <summary>
        /// Executes the maneuver burn for the configured maneuver node.
        /// </summary>
        /// <param name="computer">FlightComputer instance of the computer of the vessel the ManeuverCommand is for.</param>
        /// <param name="ctrlState">FlightCtrlState instance of the current state of the vessel.</param>
        /// <returns>true if the command has finished its work, false otherwise.</returns>
        public override bool Execute(FlightComputer computer, FlightCtrlState ctrlState)
        {
            // Halt the command if we reached our target or were command to abort by the previous tick
            if (this.RemainingDelta <= 0.01 || this.abortOnNextExecute)
            {
                this.AbortManeuver(computer);
                return true;
            }

            // Orientate vessel to maneuver prograde
            var forward = Node.GetBurnVector(computer.Vessel.orbit).normalized;
            var up = (computer.SignalProcessor.Body.position - computer.SignalProcessor.Position).normalized;
            var orientation = Quaternion.LookRotation(forward, up);
            FlightCore.HoldOrientation(ctrlState, computer, orientation, true);

            /*// This represents the theoretical acceleration but is off by a few m/s^2, probably because some parts are partially physicsless
            double thrustToMass = (FlightCore.GetTotalThrust(computer.Vessel) / computer.Vessel.GetTotalMass());
            // We need to know if the engine was activated or not to show the proper info text in the command
            if (thrustToMass == 0.0)
            {
                this.EngineActivated = false;
                return false;
            }*/

            ShipState vessel = new ShipState();
            Tally propellantsConsumed = new Tally();

            GetThrustInfo(propellantsConsumed, out totalThrust);
            double thrustToMass = totalThrust / computer.Vessel.GetTotalMass();


            //this.EngineActivated = true;

            // Before any throttling, those two values may differ from after the throttling took place
            this.RemainingDelta = this.getRemainingDeltaV(computer);

            // In case we would overpower with 100% thrust, calculate how much we actually need and set it.
            if (computer.Vessel.acceleration.magnitude > this.RemainingDelta)
            {
                // Formula which leads to this: a = ( vE – vS ) / dT
                this.throttle = this.RemainingDelta / computer.Vessel.acceleration.magnitude;
            }
                
            ctrlState.mainThrottle = (float)this.throttle;

            // TODO: THIS CAN PROBABLY BE REMOVED? RemainingDelta = this.getRemainingDeltaV(computer);

            // After throttling, the remaining time differs from beforehand (dividing delta by throttled thrustToMass)
            //this.RemainingTime = this.RemainingDelta / (ctrlState.mainThrottle * thrustToMass);

            this.RemainingTime = GetBurnTime(this.RemainingDelta);

            // We need to abort if the remaining delta was already low enough so it only takes exactly one more tick!
            double ticksRemaining = this.RemainingTime / TimeWarp.deltaTime;

            if (ticksRemaining <= 1)
            {
                this.throttle *= ticksRemaining;
                ctrlState.mainThrottle = (float)this.throttle;
                this.abortOnNextExecute = true;
                return false;
            }

            // we only compare up to the fiftieth part due to some burn-up delay when just firing up the engines
            if (this.lowestDeltaV > 0 // Do ignore the first tick
                && (this.RemainingDelta - 0.02) > this.lowestDeltaV
                && this.RemainingDelta < 1.0)   // be safe that we do not abort the command to early
            {
                // Aborting because deltaV was rising again!
                this.AbortManeuver(computer);
                return true;
            }

            // Lowest delta always has to be stored to be able to compare it in the next tick
            if (this.lowestDeltaV == 0 // Always do it on the first tick
                || this.RemainingDelta < this.lowestDeltaV)
            {
                this.lowestDeltaV = this.RemainingDelta;
            }

            return false;
        }

        /// <summary>
        /// Returns the total time for this burn in seconds
        /// </summary>
        /// <param name="f">Flightcomputer for the current vessel</param>
        /// <returns>max burn time</returns>
        public double getMaxBurnTime(FlightComputer f)
        {
            if (Node == null) return 0;

            Tally propellantsConsumed = new Tally();
            GetThrustInfo(propellantsConsumed, out totalThrust);
            return Node.DeltaV.magnitude / totalThrust / f.Vessel.GetTotalMass();

            //return Node.DeltaV.magnitude / (FlightCore.GetTotalThrust(f.Vessel) / f.Vessel.GetTotalMass());
        }

        public static ManeuverCommand WithNode(int nodeIndex, FlightComputer f)
        {
            double thrust = FlightCore.GetTotalThrust(f.Vessel);
            ManeuverNode node = f.Vessel.patchedConicSolver.maneuverNodes[nodeIndex];
            double advance = f.Delay;

            if (thrust > 0) {
                advance += (node.DeltaV.magnitude / (thrust / f.Vessel.GetTotalMass())) / 2;
                // add 1 second for the throttle down time @ the end of the burn
                advance += 1;
            }

            var newNode = new ManeuverCommand()
            {
                Node = node,
                TimeStamp = node.UT - advance,
            };
            return newNode;
        }

        /// <summary>
        /// Find the maneuver node by the saved node id (index id of the meneuver list)
        /// </summary>
        /// <param name="n">Node with the command infos</param>
        /// <param name="fc">Current flightcomputer</param>
        /// <returns>true - loaded successfull</returns>
        public override bool Load(ConfigNode n, FlightComputer fc)
        {
            if(base.Load(n,fc))
            {
                if(n.HasValue("NodeIndex"))
                {
                    this.NodeIndex = int.Parse(n.GetValue("NodeIndex"));
                    RTLog.Notify("Trying to get Maneuver {0}", this.NodeIndex);
                    if (this.NodeIndex >= 0)
                    {
                        // Set the ManeuverNode into this command
                        this.Node = fc.Vessel.patchedConicSolver.maneuverNodes[this.NodeIndex];
                        RTLog.Notify("Found Maneuver {0} with {1} dV", this.NodeIndex, this.Node.DeltaV);

                        return true;
                    }
                }
            }

            return false;
        }

        /// <summary>
        /// Save the index of the maneuver node to the persistent
        /// </summary>
        public override void Save(ConfigNode n, FlightComputer fc)
        {
            // search the node on the List
            this.NodeIndex = fc.Vessel.patchedConicSolver.maneuverNodes.IndexOf(this.Node);

            // only save this command if we are on the maneuverNode list
            if (this.NodeIndex >= 0)
            {
                base.Save(n, fc);
            }
        }

        /// <summary>
        /// This method will be triggerd right after the command was enqueued to
        /// the flight computer list.
        /// </summary>
        /// <param name="computer">Current flightcomputer</param>
        public override void CommandEnqueued(FlightComputer computer)
        {
            string KaCAddonLabel = String.Empty;
            double timetoexec = (this.TimeStamp + this.ExtraDelay) - 180;

            if (timetoexec - RTUtil.GameTime >= 0 && RTSettings.Instance.AutoInsertKaCAlerts == true)
            {
                KaCAddonLabel = computer.Vessel.vesselName + " Maneuver";

                if (RTCore.Instance != null && RTCore.Instance.kacAddon != null)
                {
                    this.KaCItemId = RTCore.Instance.kacAddon.CreateAlarm(AddOns.KerbalAlarmClockAddon.AlarmTypeEnum.Maneuver, KaCAddonLabel, timetoexec);
                }
            }

            // also add a maneuver node command to the queue
            computer.Enqueue(AttitudeCommand.ManeuverNode(timetoexec), true, true, true);
        }

        /// <summary>
        /// This method will be triggerd after deleting a command from the list.
        /// </summary>
        /// <param name="computer">Current flight computer</param>
        public override void CommandCanceled(FlightComputer computer)
        {
            // Cancel also the kac entry
            if (this.KaCItemId != String.Empty && RTCore.Instance != null && RTCore.Instance.kacAddon != null)
            {
                RTCore.Instance.kacAddon.DeleteAlarm(this.KaCItemId);
                this.KaCItemId = String.Empty;
            }
        }
    }
}
