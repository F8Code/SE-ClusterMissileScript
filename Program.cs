using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Text;
using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using VRage;
using VRage.Collections;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.GUI.TextPanel;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ObjectBuilders.Definitions;
using VRageMath;
using static IngameScript.Program.Missile;

namespace IngameScript
{
    partial class Program : MyGridProgram
    {
        //PRE-NAVIGATION SETTINGS
        double freeFallTime = 0.0;
        double noGuidanceFlightTime = 1.0;

        //NAVIGATION MODE
        int navigationType = 0;
        // 0 = GlidingUntilTargetDetected
        // 1 = BeamRiding
        // 2 = GPSUnlessTargetDetected
        // 3 = AbsoluteGPS

        //NAVIGATION SETTINGS
        double gpsX = 0;
        double gpsY = 0;
        double gpsZ = 0;
        string beamBlockCustomDataTag = "beam";

        //TARGET CHARACTERISTICS
        int missileTargetType = 0;
        // 0 = Large Grid
        // 1 = Small Grid 
        // 2 = Planet
        // 3 = Asteroid
        // 4 = Floating Object
        // 5 = Any
        int missileTargetRelationship = 6;
        // 0 = Enemies
        // 1 = No Ownership
        // 2 = Neutral
        // 3 = Owner
        // 4 = Faction Share
        // 5 = Friends
        // 6 = Any

        //WORLD SETTINGS
        int maxSmallGridSpeed = 100;

        //MISSILE BUILD SETTINGS
        string missileDockingMergesNameTag = "TMM1";
        string missileDockingConnectorsNameTag = "TMC1";
        int missileDockingConnectorsCount = 4;

        //FLIGHT SETTINGS
        double sideMomentumCorrection = 0.1;
        double downMomentumCorrection = 0.2;
        double upMomentumCorrection = 0.1;
        double turnSpeedModifier = 1.0;
        double fuelSavingMultiplier = 4.0;

        //COLLISION AVOIDANCE SETTINGS
        double obstacleEvasionStrength = 1.0;
        double maxAvoidanceAngle = 30.0;

        //RADAR SETTINGS
        int radarDistance = 4000;
        int radarConeAngle = 30;
        int radarResolution = 50;
        double trackingOvershootFactor = 1.1;
        double trackingAreaModifier = 1.25;

        //BOMBARDMENT SETTINGS
        int minimumBombardmentAngle = 0;
        double acceptableInaccuracyModifier = 1.0;
        int proximityToStartBombardment = 3000;
        int overshootCorrection = 25;
        int carpetBombingLength = 0;

        double deadManSwitchTime = 30.0;
        int distanceFromTargetToArm = 250;
        double detonationTimeDelay = 0.0;

        //PROGRAM STARTS HERE!

        readonly Dictionary<int, Missile.GuidanceTypes> NavigationTypeMapping = new Dictionary<int, Missile.GuidanceTypes>
        {
            { 0, Missile.GuidanceTypes.GlidingUntilTargetDetected },
            { 1, Missile.GuidanceTypes.BeamRiding },
            { 2, Missile.GuidanceTypes.GPSUnlessTargetDetected },
            { 3, Missile.GuidanceTypes.AbsoluteGPS }
        };

        readonly Dictionary<int, MyDetectedEntityType?> TargetTypeMapping = new Dictionary<int, MyDetectedEntityType?>
        {
            { 0, MyDetectedEntityType.LargeGrid },
            { 1, MyDetectedEntityType.SmallGrid },
            { 2, MyDetectedEntityType.Planet },
            { 3, MyDetectedEntityType.Asteroid },
            { 4, MyDetectedEntityType.FloatingObject },
            { 5, null }
        };

        readonly Dictionary<int, MyRelationsBetweenPlayerAndBlock?> TargetRelationshipMapping = new Dictionary<int, MyRelationsBetweenPlayerAndBlock?>
        {
            { 0, MyRelationsBetweenPlayerAndBlock.Enemies },
            { 1, MyRelationsBetweenPlayerAndBlock.NoOwnership },
            { 2, MyRelationsBetweenPlayerAndBlock.Neutral },
            { 3, MyRelationsBetweenPlayerAndBlock.Owner },
            { 4, MyRelationsBetweenPlayerAndBlock.FactionShare },
            { 5, MyRelationsBetweenPlayerAndBlock.Friends },
            { 6, null }
        };

        void NormalizeSettings()
        {
            if (!NavigationTypeMapping.ContainsKey(navigationType))
                navigationType = 0;
            missileGuidance = NavigationTypeMapping[navigationType];

            if (!TargetTypeMapping.ContainsKey(missileTargetType))
                missileTargetType = 0;
            targetType = TargetTypeMapping[missileTargetType];

            if (!TargetRelationshipMapping.ContainsKey(missileTargetRelationship))
                missileTargetRelationship = 0;
            targetRelationship = TargetRelationshipMapping[missileTargetRelationship];

            freeFallTime = Math.Max(0, freeFallTime);
            noGuidanceFlightTime = Math.Max(0.5, noGuidanceFlightTime);
            maxSmallGridSpeed = Math.Max(1, maxSmallGridSpeed);
            sideMomentumCorrection = Math.Max(0, sideMomentumCorrection);
            downMomentumCorrection = Math.Max(0, downMomentumCorrection);
            upMomentumCorrection = Math.Max(0, upMomentumCorrection);
            turnSpeedModifier = MathHelper.Max(0.1, turnSpeedModifier);
            fuelSavingMultiplier = Math.Max(1, fuelSavingMultiplier);
            obstacleEvasionStrength = Math.Max(0, obstacleEvasionStrength);
            maxAvoidanceAngle = MathHelper.Clamp(maxAvoidanceAngle, 0, 90);
            radarDistance = Math.Max(1000, radarDistance);
            radarConeAngle = MathHelper.Clamp(radarConeAngle, 0, 90);
            radarResolution = Math.Max(10, radarResolution);
            trackingOvershootFactor = Math.Max(1, trackingOvershootFactor);
            trackingAreaModifier = Math.Max(0.1, trackingAreaModifier);
            distanceFromTargetToArm = Math.Max(0, distanceFromTargetToArm);
            detonationTimeDelay = Math.Max(0, detonationTimeDelay);
            minimumBombardmentAngle = MathHelper.Clamp(minimumBombardmentAngle, 0, 89);
            deadManSwitchTime = Math.Max(0, deadManSwitchTime);
            carpetBombingLength = MathHelper.Clamp(carpetBombingLength, 0, 150);
            acceptableInaccuracyModifier = MathHelper.Clamp(acceptableInaccuracyModifier, 0.1, 10);
            proximityToStartBombardment = MathHelper.Clamp(proximityToStartBombardment, 1000, 9999);
            overshootCorrection = MathHelper.Clamp(overshootCorrection, 0, 100);
        }

        public Program()
        {
            program = this;
            Runtime.UpdateFrequency = UpdateFrequency.Update100;

            NormalizeSettings();

            Load();

            TrySetupMissileConnection();
        }

        public void Save()
        {
            string gpsData = $"{gpsX}:{gpsY}:{gpsZ}";

            Storage = gpsData;
        }

        void Load()
        {
            if (Storage.Length == 0 || gpsX != 0 || gpsY != 0 || gpsZ != 0) return;

            string[] gpsData = Storage.Split(':');

            double x, y, z;
            if (double.TryParse(gpsData[0], out x) && double.TryParse(gpsData[1], out y) && double.TryParse(gpsData[2], out z))
            {
                gpsX = x;
                gpsY = y;
                gpsZ = z;
            }
        }

        MyGridProgram program;

        //SETTING VARS
        Missile.GuidanceTypes missileGuidance;
        MyDetectedEntityType? targetType;
        MyRelationsBetweenPlayerAndBlock? targetRelationship;

        //MISSILE CONNECTION POINTS
        List<IMyShipMergeBlock> missileDockingMerges = new List<IMyShipMergeBlock>();
        List<IMyShipConnector> missileDockingConnectors = new List<IMyShipConnector>();

        //THE MISSILE ITSELF!
        Missile missile;

        //LOGIC RESPONSIBLE VARS
        bool isMissileLaunched = false;
        IMyTerminalBlock beamBlock;

        public void Main(string argument, UpdateType updateSource)
        {
            if ((updateSource & UpdateType.Update100) != 0)
                TrySetupMissileConnection();

            if (argument.Contains(":"))
                TryParseGps(argument);

            if (isMissileLaunched && !missile.Update())
                TerminateMissile();

            if (argument.ToLower() == "launch" && !isMissileLaunched)
                LaunchMissile();

            UpdateBeamGuidanceData();
            
            EchoScriptState();
        }

        void TrySetupMissileConnection()
        {
            List<IMyShipMergeBlock> allMerges = new List<IMyShipMergeBlock>();
            GridTerminalSystem.GetBlocksOfType(allMerges);
            missileDockingMerges = allMerges.Where(block => block.CustomName.Contains(missileDockingMergesNameTag)).ToList();

            List<IMyShipConnector> allConnectors = new List<IMyShipConnector>();
            GridTerminalSystem.GetBlocksOfType(allConnectors);
            missileDockingConnectors = allConnectors.Where(block => block.CustomName.Contains(missileDockingConnectorsNameTag)).ToList();

            foreach (IMyShipConnector connector in missileDockingConnectors)
                connector.Connect();

            if (missileDockingConnectors.Count == missileDockingConnectorsCount)
                Runtime.UpdateFrequency = UpdateFrequency.Once;
        }

        void LaunchMissile()
        {
            if (navigationType == 1 && beamBlock == null) 
                return;

            if ((navigationType == 2 || navigationType == 3) && gpsX == 0 && gpsY == 0 && gpsZ == 0) 
                return;

            foreach (IMyShipMergeBlock merge in missileDockingMerges)
                merge.Enabled = false;

            List<IMyTerminalBlock> allBlocks = new List<IMyTerminalBlock>();
            GridTerminalSystem.GetBlocks(allBlocks);

            List<IMyTerminalBlock> missileBlocks = allBlocks.Where(
                block => block.CubeGrid == missileDockingMerges[0].CubeGrid && !block.CustomName.Contains(missileDockingMergesNameTag)
            ).ToList();

            missile = new Missile(program, missileBlocks);

            if(!missile.CanMissileOperate())
            {
                TerminateMissile();
                return;
            }

            foreach (IMyShipConnector connector in missileDockingConnectors)
                connector.Disconnect();

            foreach (IMyShipConnector connector in missileDockingConnectors)
                connector.Enabled = false;

            SetMissileSettings();

            Runtime.UpdateFrequency = UpdateFrequency.Update10;
            isMissileLaunched = true;
        }

        void SetMissileSettings()
        {
            missile.freeFallTime = freeFallTime;
            missile.noGuidanceFlightTime = noGuidanceFlightTime;
            missile.gpsCoordinates = new Vector3D(gpsX, gpsY, gpsZ);
            missile.missileGuidance = missileGuidance;
            missile.acceptableInaccuracyModifier = acceptableInaccuracyModifier;
            missile.bombardmentProximity = proximityToStartBombardment;
            missile.overshootCorrection = overshootCorrection;

            missile.radar.targetType = targetType;
            missile.radar.targetRelationship = targetRelationship;
            missile.radar.distance = radarDistance;
            missile.radar.scanConeAngle = radarConeAngle;
            missile.radar.resolution = radarResolution;
            missile.radar.trackingOvershootFactor = trackingOvershootFactor;
            missile.radar.trackingAreaModifier = trackingAreaModifier;

            missile.flightComputer.sideMomentumCorrectionFactor = (float)sideMomentumCorrection;
            missile.flightComputer.downMomentumCorrectionFactor = (float)downMomentumCorrection;
            missile.flightComputer.upMomentumCorrectionFactor = (float)upMomentumCorrection;
            missile.flightComputer.turnSpeedModifier = (float)turnSpeedModifier;
            missile.flightComputer.fuelSavingMultiplier = (float)fuelSavingMultiplier;
            missile.flightComputer.obstacleEvasionStrength = obstacleEvasionStrength;
            missile.flightComputer.maxAvoidanceAngle = maxAvoidanceAngle;

            missile.bombingComputer.maxSmallGridSpeed = maxSmallGridSpeed;
            missile.bombingComputer.distanceFromTargetToArm = distanceFromTargetToArm;
            missile.bombingComputer.detonationTimeDelay = detonationTimeDelay;
            missile.bombingComputer.minimumBombardmentAngle = minimumBombardmentAngle;
            missile.bombingComputer.deadManSwitchTime = deadManSwitchTime;
            missile.bombingComputer.carpetBombingLength = carpetBombingLength;
        }

        void UpdateBeamGuidanceData()
        {
            if (navigationType != 1) return;

            if (missile == null)
            {
                if (beamBlock != null) return;

                List<IMyTerminalBlock> allBlocks = new List<IMyTerminalBlock>();
                GridTerminalSystem.GetBlocks(allBlocks);
                List<IMyTerminalBlock> blocksWithBeamTag = allBlocks.Where(block => block.CustomData.Contains(beamBlockCustomDataTag)).ToList();

                if (blocksWithBeamTag.Count == 1)
                    beamBlock = blocksWithBeamTag[0];

                return;
            }

            if (navigationType == 1 && beamBlock == null)
            {
                navigationType = 0;
                missile.missileGuidance = Missile.GuidanceTypes.GlidingUntilTargetDetected;
                return;
            }

            Vector3D beamBlockForward;
            IMyLargeTurretBase turret = beamBlock as IMyLargeTurretBase;
            if (turret != null)
            {
                Vector3D.CreateFromAzimuthAndElevation(turret.Azimuth, turret.Elevation, out beamBlockForward);
                beamBlockForward = Vector3D.TransformNormal(beamBlockForward, beamBlock.WorldMatrix);
            }
            else
                beamBlockForward = beamBlock.WorldMatrix.Forward;

            Vector3D missileToBeam = missile.missileData.position - beamBlock.GetPosition();
            Vector3D missileRayProjectedPos = Vector3D.ProjectOnVector(ref missileToBeam, ref beamBlockForward);

            missile.gpsCoordinates = beamBlock.GetPosition() + missileRayProjectedPos + beamBlockForward * 1000;
        }

        void TerminateMissile()
        {
            missile = null;
            Runtime.UpdateFrequency = UpdateFrequency.Update100;
            isMissileLaunched = false;

            foreach (IMyShipMergeBlock merge in missileDockingMerges)
                merge.Enabled = true;
        }

        void EchoScriptState()
        {
            switch (Runtime.UpdateFrequency)
            {
                case UpdateFrequency.Update100:
                    Echo("Missile is being built.");
                    break;
                case UpdateFrequency.None:
                    Echo("Missile is ready for launch!");
                    break;
                case UpdateFrequency.Update10:
                    Echo("Missile is traveling towards target.");
                    break;
                case UpdateFrequency.Update1:
                    Echo("Missile is bombarding the target!");
                    break;
            }

            Echo("Misile guidance: " + missileGuidance.ToString());

            if (navigationType != 3)
                Echo($"Target characteristics: {targetType?.ToString() ?? "Any"}, {targetRelationship?.ToString() ?? "Any"}");

            if (navigationType == 1)
            {
                if (beamBlock == null)
                {
                    List<IMyTerminalBlock> allBlocks = new List<IMyTerminalBlock>();
                    GridTerminalSystem.GetBlocks(allBlocks);
                    List<IMyTerminalBlock> blocksWithBeamTag = allBlocks.Where(block => block.CustomData.Contains(beamBlockCustomDataTag)).ToList();

                    if (blocksWithBeamTag.Count() == 0)
                        WriteError("No block with custom data beam tag found.");
                    else if (blocksWithBeamTag.Count() > 1)
                    {
                        WriteError("More than one block with custom data beam tag found.");
                        Echo("Blocks with beam tag:");
                        foreach (IMyTerminalBlock block in blocksWithBeamTag)
                            Echo(block.CustomName);
                    }
                }
                else
                    Echo("Beam block: " + beamBlock.CustomName);
            }
            else if (navigationType == 2 || navigationType == 3)
            {
                if (gpsX == 0 && gpsY == 0 && gpsZ == 0)
                    WriteError("No gps coordinates have been set.");
                else
                    Echo($"Selected GPS: {gpsX}, {gpsY}, {gpsZ}");
            }
        }

        void WriteError(string error)
        {
            Echo("\n!WARNING!");
            Echo(error);
            Echo("If not resolved the missile won't launch!");
        }

        void TryParseGps(string arg)
        {
            string[] splitString = arg.Split(':');

            double x, y, z;
            if (double.TryParse(splitString[2], out x) && double.TryParse(splitString[3], out y) && double.TryParse(splitString[4], out z))
            {
                gpsX = x;
                gpsY = y;
                gpsZ = z;
            }

            if(missile != null)
                missile.gpsCoordinates = new Vector3D(gpsX, gpsY, gpsZ);
        }

        public class Missile
        {
            //Flight settings
            public double freeFallTime;
            public double noGuidanceFlightTime;

            //Guidance settings
            public GuidanceTypes missileGuidance = GuidanceTypes.AbsoluteGPS;
            public Vector3D gpsCoordinates;

            //Bombardment settings
            public double acceptableInaccuracyModifier = 1.0;
            public int bombardmentProximity = 4000;
            public int overshootCorrection = 25;

            //Accessible
            public MissileData missileData;

            //Inner classes
            public readonly StateMachine stateMachine;
            public readonly FlightComputer flightComputer;
            public readonly Radar radar;
            public readonly BombingComputer bombingComputer;

            //In-game Blocks
            readonly List<IMyRemoteControl> remotes = new List<IMyRemoteControl>();
            readonly List<IMyBatteryBlock> batteries = new List<IMyBatteryBlock>();
            readonly List<IMyGasTank> tanks = new List<IMyGasTank>();
            readonly List<IMyThrust> thrusters = new List<IMyThrust>();
            readonly List<IMyGyro> gyros = new List<IMyGyro>();
            readonly List<IMyCameraBlock> cameras = new List<IMyCameraBlock>();
            readonly List<IMyShipMergeBlock> merges = new List<IMyShipMergeBlock>();
            readonly List<IMyWarhead> warheads = new List<IMyWarhead>();

            //Logic
            Vector3D originalPos;
            Vector3D originalForwardVec;
            bool targetLocked = false;
            double timeLocked = 0;
            MyDetectedEntityInfo lastDetectedEnemyInfo;

            //Other
            readonly MyGridProgram Program;

            public class MissileData
            {
                public MatrixD worldMatrix;
                public Vector3D position;
                public Vector3D linearVelocity;
                public Vector3D angularVelocity;
                public double mass;
                public Vector3D gravity;
                public double elapsedTime = 0;
            }

            public enum GuidanceTypes
            {
                BeamRiding,
                GlidingUntilTargetDetected,
                GPSUnlessTargetDetected,
                AbsoluteGPS,
            }

            public Missile(MyGridProgram Program, List<IMyTerminalBlock> missileBlocks)
            {
                this.Program = Program;

                this.remotes = missileBlocks.OfType<IMyRemoteControl>().ToList();
                if (remotes.Count == 0) return;
                this.batteries = missileBlocks.OfType<IMyBatteryBlock>().ToList();
                this.tanks = missileBlocks.OfType<IMyGasTank>().ToList();
                this.thrusters = missileBlocks.OfType<IMyThrust>().ToList();
                this.gyros = missileBlocks.OfType<IMyGyro>().ToList();
                this.cameras = missileBlocks.OfType<IMyCameraBlock>().ToList();
                this.merges = missileBlocks.OfType<IMyShipMergeBlock>().Where(merge => Vector3D.Dot(merge.WorldMatrix.Up, remotes[0].WorldMatrix.Backward) > 0.9 || Vector3D.Dot(merge.WorldMatrix.Right, remotes[0].WorldMatrix.Backward) > 0.9).ToList();
                this.warheads = missileBlocks.OfType<IMyWarhead>().ToList();

                RemoveAllDestroyedBlocks();

                stateMachine = new StateMachine(Program, this); //Needs to be declared before CanMissileOperate()
                if (!CanMissileOperate()) return;

                missileData = new MissileData();
                UpdateMissileData();

                missileData.elapsedTime = 0;
                originalPos = missileData.position;
                originalForwardVec = missileData.worldMatrix.Forward;

                flightComputer = new FlightComputer(Program, missileData, thrusters, gyros);
                radar = new Radar(Program, missileData, cameras);
                bombingComputer = new BombingComputer(Program, missileData, merges, warheads);

                SetupBlocks();
            }

            void SetupBlocks()
            {
                foreach (IMyBatteryBlock battery in batteries)
                {
                    battery.Enabled = true;
                    battery.ChargeMode = ChargeMode.Auto;
                }

                foreach (IMyGasTank tank in tanks)
                {
                    tank.Enabled = true;
                    tank.Stockpile = false;
                }

                foreach (IMyThrust thruster in thrusters)
                {
                    thruster.Enabled = false;
                    thruster.ThrustOverride = 0;
                }

                foreach (IMyGyro gyro in gyros)
                {
                    gyro.Enabled = true;
                    gyro.GyroOverride = true;
                    gyro.Pitch = gyro.Yaw = gyro.Roll = 0;
                }

                foreach (IMyCameraBlock camera in cameras)
                {
                    camera.Enabled = true;
                    camera.EnableRaycast = true;
                }

                foreach (IMyWarhead warhead in warheads)
                {
                    warhead.IsArmed = false;
                }
            }

            public bool Update()
            {
                RemoveAllDestroyedBlocks();
                if (!CanMissileOperate())
                    return false;

                UpdateStateMachineConditions();
                stateMachine.UpdateMissileState(missileGuidance);
                if (stateMachine.missileState != StateMachine.MissileStates.BombHandling)
                {
                    UpdateMissileData();
                    radar.Update(missileGuidance == GuidanceTypes.AbsoluteGPS);
                }

                HandleMissileState(stateMachine.missileState);

                return true;
            }

            void RemoveAllDestroyedBlocks()
            {
                RemoveDestroyed(remotes);
                RemoveDestroyed(batteries);
                RemoveDestroyed(tanks);
                RemoveDestroyed(thrusters);
                RemoveDestroyed(gyros);
                RemoveDestroyed(cameras);
                RemoveDestroyed(merges);
                RemoveDestroyed(warheads);
            }

            void RemoveDestroyed<T>(List<T> blocks) where T : class, IMyTerminalBlock
            {
                blocks.RemoveAll(block => block == null || !block.IsFunctional);
            }

            public bool CanMissileOperate()
            {
                if(stateMachine.missileState != StateMachine.MissileStates.BombHandling)
                {
                    if (!remotes.Any()) return false;
                    if (!gyros.Any()) return false;
                    if (!thrusters.Any()) return false;
                    if (remotes[0].GetPosition().IsZero()) return false; //Missile deleted from world

                    if (stateMachine.missileState != StateMachine.MissileStates.TargetBombardment)
                    {
                        if (missileGuidance != GuidanceTypes.GPSUnlessTargetDetected && missileGuidance != GuidanceTypes.AbsoluteGPS && cameras.Count <= 1) return false;
                        if (!merges.Any()) return false;
                    }
                }
                if (!warheads.Any()) return false;

                return true;
            }

            void UpdateMissileData()
            {
                missileData.worldMatrix = remotes[0].WorldMatrix;
                missileData.position = remotes[0].CubeGrid.GetPosition();
                missileData.linearVelocity = remotes[0].GetShipVelocities().LinearVelocity;
                missileData.angularVelocity = remotes[0].GetShipVelocities().AngularVelocity;
                missileData.mass = remotes[0].CalculateShipMass().BaseMass;
                missileData.gravity = remotes[0].GetNaturalGravity();
                missileData.elapsedTime += Program.Runtime.TimeSinceLastRun.TotalSeconds;
            }

            void UpdateStateMachineConditions()
            {
                if (missileData.elapsedTime > freeFallTime)
                    stateMachine.stateConditions.FreefallTimePassed = true;

                if (missileData.elapsedTime - freeFallTime > noGuidanceFlightTime)
                    stateMachine.stateConditions.NoGuidanceFlightTimePassed = true;

                stateMachine.stateConditions.TargetWithMatchingCharacteristicsDetected = !radar.detectedGrid.IsEmpty();

                if (Vector3D.Distance(CalculateTargetInterceptionPoint(), missileData.position) <= bombardmentProximity)
                    stateMachine.stateConditions.TargetWithinBombardmentProximity = true;

                if (!merges.Any())
                    stateMachine.stateConditions.BombsHaveBeenReleased = true;

                if (!warheads.Any())
                    stateMachine.stateConditions.AllBombsHaveExploded = true;
            }

            void HandleMissileState(StateMachine.MissileStates missileState)
            {
                switch (missileState)
                {
                    case StateMachine.MissileStates.Freefall:
                        HandleFreefallState();
                        break;
                    case StateMachine.MissileStates.NoGuidanceFlight:
                        HandleNoGuidanceFlightState();
                        break;
                    case StateMachine.MissileStates.GuidanceFlight:
                        HandleGuidanceFlightState(missileGuidance);
                        break;
                    case StateMachine.MissileStates.DetectedTargetFlight:
                        HandleDetectedTargetFlightState();
                        break;
                    case StateMachine.MissileStates.TargetBombardment:
                        HandleTargetBombardmentState();
                        break;
                    case StateMachine.MissileStates.BombHandling:
                        HandleBombHandlingState();
                        break;
                }
            }

            void HandleFreefallState() { }

            void HandleNoGuidanceFlightState()
            {
                HandleGuidanceFlightState(GuidanceTypes.GlidingUntilTargetDetected);
            }

            void HandleGuidanceFlightState(GuidanceTypes guidance)
            {
                Vector3D target = Vector3D.Zero;
                switch (guidance)
                {
                    case GuidanceTypes.GlidingUntilTargetDetected:
                        Vector3D missileTraveledVector = missileData.position - originalPos;
                        if (Vector3D.Dot(missileTraveledVector, originalForwardVec) < 0)
                            missileTraveledVector *= -1;
                        target = originalPos + Vector3D.ProjectOnVector(ref missileTraveledVector, ref originalForwardVec) + originalForwardVec * 1000;
                        break;
                    case GuidanceTypes.BeamRiding:
                    case GuidanceTypes.GPSUnlessTargetDetected:
                    case GuidanceTypes.AbsoluteGPS:
                        target = gpsCoordinates;
                        break;
                }

                flightComputer.AdjustThrustOverride();
                flightComputer.FlyTo(flightComputer.CalculateTargetPath(target, radar.obstaclePoints));
            }

            void HandleDetectedTargetFlightState()
            {
                lastDetectedEnemyInfo = radar.detectedGrid;

                Vector3D target = CalculateTargetInterceptionPoint();

                flightComputer.AdjustThrustOverride();
                flightComputer.FlyTo(flightComputer.CalculateTargetPath(target, radar.obstaclePoints));
            }

            void HandleTargetBombardmentState()
            {
                if (!radar.detectedGrid.IsEmpty())
                    lastDetectedEnemyInfo = radar.detectedGrid;

                Vector3D target = CalculateTargetInterceptionPoint();
                target = bombingComputer.AddTargetOvershootCorrection(bombingComputer.ModifyTargetForCarpetBombing(target), overshootCorrection);

                bombingComputer.UpdateBombardmentData(target);
                Vector3D targetHeading = bombingComputer.CalculateBombardmentPath(target, lastDetectedEnemyInfo.IsEmpty() || lastDetectedEnemyInfo.Type == MyDetectedEntityType.Planet ? 45 : 30);

                flightComputer.AdjustThrustOverride(true);
                flightComputer.FlyTo(targetHeading);

                double lockOnTime = 3 * Vector3D.Distance(missileData.position, target) / 3000;
                if (!targetLocked)
                {
                    if (Vector3D.Distance(bombingComputer.bombardmentCenter, target) < CalculateAcceptableBombardmentError(target) * acceptableInaccuracyModifier)
                        timeLocked += Program.Runtime.TimeSinceLastRun.TotalSeconds;
                    else
                        timeLocked = 0;

                    if (timeLocked > lockOnTime)
                        targetLocked = true;

                    return;
                }

                timeLocked += Program.Runtime.TimeSinceLastRun.TotalSeconds;

                float bombardmentRoll = bombingComputer.CalculateBombardmentRoll(lastDetectedEnemyInfo, warheads[0].Mass < 500 ? 4.6 : 22.98);
                foreach (IMyGyro gyro in gyros)
                    gyro.Roll += bombardmentRoll;

                if (Math.Abs(bombardmentRoll) < 10 || timeLocked > lockOnTime + 2)
                    bombingComputer.SplitNextMerges(2);
            }

            void HandleBombHandlingState()
            {
                Vector3D target = CalculateTargetInterceptionPoint();

                bombingComputer.ManageBombExplosions(target);
            }

            Vector3D CalculateTargetInterceptionPoint()
            {
                if (missileGuidance == GuidanceTypes.AbsoluteGPS || missileGuidance == GuidanceTypes.GPSUnlessTargetDetected && lastDetectedEnemyInfo.IsEmpty())
                    return gpsCoordinates;

                if (lastDetectedEnemyInfo.IsEmpty())
                    return Vector3D.Zero;

                if (lastDetectedEnemyInfo.Type == MyDetectedEntityType.Asteroid || lastDetectedEnemyInfo.Type == MyDetectedEntityType.Planet)
                {
                    Vector3D target = Vector3D.Zero;
                    foreach (Vector3D terrain in radar.terrainPoints)
                        target += terrain;

                    return target / radar.terrainPoints.Count();
                }

                return flightComputer.CalculateTargetInterceptionPoint(lastDetectedEnemyInfo, radar.timeSinceLastDetection);
            }

            int CalculateAcceptableBombardmentError(Vector3D target)
            {
                double distanceToTarget = Vector3D.Distance(target, missileData.position);

                if (missileGuidance != GuidanceTypes.AbsoluteGPS && lastDetectedEnemyInfo.Velocity.Length() > 0)
                    return (int)Math.Max(10, Math.Max(1, 3000 / distanceToTarget) * lastDetectedEnemyInfo.Velocity.Length() / 2);

                return (int)Math.Max(5, 5 * (1500 / distanceToTarget));
            }

            public class StateMachine
            {
                //Logic settings
                public StatesSwitchConditions stateConditions = new StatesSwitchConditions();

                //Accesible
                public MissileStates missileState { get; private set; } = MissileStates.Freefall;

                //Other
                MyGridProgram Program;
                Missile missile;

                public enum MissileStates
                {
                    Freefall,
                    NoGuidanceFlight,
                    GuidanceFlight,
                    DetectedTargetFlight,
                    TargetBombardment,
                    BombHandling,
                }

                public struct StatesSwitchConditions
                {
                    public bool FreefallTimePassed;
                    public bool NoGuidanceFlightTimePassed;
                    public bool TargetWithMatchingCharacteristicsDetected;
                    public bool TargetWithinBombardmentProximity;
                    public bool BombsHaveBeenReleased;
                    public bool AllBombsHaveExploded;
                }

                public StateMachine(MyGridProgram Program, Missile missile)
                {
                    this.Program = Program;
                    this.missile = missile;
                }

                public void UpdateMissileState(Missile.GuidanceTypes missileGuidance)
                {
                    switch (missileState)
                    {
                        case MissileStates.Freefall:
                            if (stateConditions.FreefallTimePassed)
                            {
                                OnNoGuidanceFlightStateTransition();
                                missileState = MissileStates.NoGuidanceFlight;
                            }
                            break;
                        case MissileStates.NoGuidanceFlight:
                            if (stateConditions.NoGuidanceFlightTimePassed)
                            {
                                OnGuidanceFlightStateTransition();
                                missileState = MissileStates.GuidanceFlight;
                            }
                            break;
                        case MissileStates.GuidanceFlight:
                            switch (missileGuidance)
                            {
                                case GuidanceTypes.BeamRiding:
                                case GuidanceTypes.GlidingUntilTargetDetected:
                                case GuidanceTypes.GPSUnlessTargetDetected:
                                    if (stateConditions.TargetWithMatchingCharacteristicsDetected)
                                    {
                                        OnDetectedTargetFlightStateTransition();
                                        missileState = MissileStates.DetectedTargetFlight;
                                    }
                                    break;
                                case GuidanceTypes.AbsoluteGPS:
                                    if (stateConditions.TargetWithinBombardmentProximity)
                                    {
                                        OnTargetBombardmentStateTransition();
                                        missileState = MissileStates.TargetBombardment;
                                    }
                                    break;
                            }
                            break;
                        case MissileStates.DetectedTargetFlight:
                            if (stateConditions.TargetWithinBombardmentProximity)
                            {
                                OnTargetBombardmentStateTransition();
                                missileState = MissileStates.TargetBombardment;
                            }
                            else if (!stateConditions.TargetWithMatchingCharacteristicsDetected)
                            {
                                OnGuidanceFlightStateTransition();
                                missileState = MissileStates.GuidanceFlight;
                            }
                            break;
                        case MissileStates.TargetBombardment:
                            if (stateConditions.BombsHaveBeenReleased)
                            {
                                OnBombHandlingStateTransition();
                                missileState = MissileStates.BombHandling;
                            }
                            break;
                        case MissileStates.BombHandling:
                            if (stateConditions.AllBombsHaveExploded) { }
                            break;
                    }
                }

                void OnNoGuidanceFlightStateTransition() 
                {
                    foreach(IMyThrust thruster in missile.thrusters)
                        thruster.Enabled = true;
                }
                void OnGuidanceFlightStateTransition() { }
                void OnDetectedTargetFlightStateTransition() { }

                void OnTargetBombardmentStateTransition()
                {
                    Program.Runtime.UpdateFrequency = UpdateFrequency.Update1;

                    missile.flightComputer.upMomentumCorrectionFactor = missile.flightComputer.downMomentumCorrectionFactor = missile.flightComputer.sideMomentumCorrectionFactor = 0.25F;
                    missile.flightComputer.turnSpeedModifier = 1;
                }

                void OnBombHandlingStateTransition()
                {
                    foreach (IMyThrust thruster in missile.thrusters)
                        thruster.Enabled = false;

                    foreach (IMyGyro gyro in missile.gyros)
                        gyro.Yaw = gyro.Pitch = gyro.Roll = 0;
                }
            }

            public class FlightComputer
            {
                //Collision avoidance settings
                public double obstacleEvasionStrength { set { pathFinder.obstacleEvasionStrength = value; } }
                public double maxAvoidanceAngle { set { pathFinder.maxAvoidanceAngle = value; } }

                //Flight settings
                public float fuelSavingMultiplier = 2;
                public float turnSpeedModifier { set { orientationModule.turnSpeedModifier = value; } }
                public float sideMomentumCorrectionFactor { set { orientationModule.sideMomentumCorrectionFactor = value; } }
                public float downMomentumCorrectionFactor { set { orientationModule.downMomentumCorrectionFactor = value; } }
                public float upMomentumCorrectionFactor { set { orientationModule.upMomentumCorrectionFactor = value; } }

                //Inner classes
                readonly PathFinderModule pathFinder;
                readonly OrientationModule orientationModule;

                //In-game Blocks
                readonly List<IMyThrust> thrusters = new List<IMyThrust>();

                //Logic
                readonly Missile.MissileData missileData;

                //Other
                readonly MyGridProgram Program;

                public FlightComputer(MyGridProgram Program, Missile.MissileData missileData, List<IMyThrust> thrusters, List<IMyGyro> gyros)
                {
                    this.Program = Program;

                    this.thrusters = thrusters;
                    this.missileData = missileData;

                    this.pathFinder = new PathFinderModule(Program, missileData);
                    this.orientationModule = new OrientationModule(Program, missileData, thrusters, gyros);
                }

                public void AdjustThrustOverride(bool maxOverride = false)
                {
                    if (missileData.linearVelocity.Length() < 99 || maxOverride)
                    {
                        foreach (IMyThrust thruster in thrusters)
                            thruster.ThrustOverride = thruster.MaxThrust;
                        return;
                    }

                    if (missileData.gravity.IsZero())
                    {
                        foreach (IMyThrust thruster in thrusters)
                            thruster.ThrustOverride = Math.Max(thruster.MaxThrust * 0.1F, thruster.MaxThrust / fuelSavingMultiplier);
                        return;
                    }

                    foreach (IMyThrust thruster in thrusters)
                    {
                        double gravityCorrectionThrust = missileData.gravity.Length() * missileData.mass / thrusters.Count;
                        thruster.ThrustOverride = (float)Math.Max(gravityCorrectionThrust * 2, thruster.MaxThrust / fuelSavingMultiplier);
                    }
                }

                public Vector3D CalculateTargetPath(Vector3D target, List<Vector3D> obstaclePoints = null)
                {
                    return pathFinder.CalculateTargetPath(target, obstaclePoints);
                }

                public Vector3D CalculateTargetInterceptionPoint(MyDetectedEntityInfo target, double timeSinceLastTargetDetection)
                {
                    return pathFinder.CalculateTargetInterceptionPoint(target, timeSinceLastTargetDetection);
                }

                public void FlyTo(Vector3D target)
                {
                    orientationModule.FlyTo(target);
                }

                public class PathFinderModule
                {
                    //Collision avoidance settings
                    public double obstacleEvasionStrength = 100;
                    public double maxAvoidanceAngle = 30;

                    //Logic
                    readonly Missile.MissileData missileData;

                    //Other
                    readonly MyGridProgram Program;

                    public PathFinderModule(MyGridProgram Program, Missile.MissileData missileData)
                    {
                        this.Program = Program;

                        this.missileData = missileData;
                    }

                    public Vector3D CalculateTargetPath(Vector3D target, List<Vector3D> obstaclePoints)
                    {
                        Vector3D targetDir = target - missileData.position;
                        Vector3D avoidanceVector = Vector3D.Zero;

                        foreach (Vector3D obstacle in obstaclePoints)
                        {
                            Vector3D obstacleDir = obstacle - missileData.position;

                            if (Vector3D.Dot(obstacleDir, target - obstacle) < 0) continue;

                            Vector3D missilePosClosestToObstacle = missileData.position + Vector3D.ProjectOnVector(ref obstacleDir, ref targetDir);
                            Vector3D avoidanceDir = missilePosClosestToObstacle - obstacle;

                            if (avoidanceDir.IsZero()) continue;

                            avoidanceVector += avoidanceDir.Normalized() * obstacleEvasionStrength * 10000 / Vector3D.Distance(missileData.position, obstacle);
                        }

                        double maxAvoidance = Math.Tan(MathHelper.ToRadians(maxAvoidanceAngle));
                        if (avoidanceVector.Length() / targetDir.Length() > maxAvoidance)
                            avoidanceVector = avoidanceVector.Normalized() * targetDir.Length() * maxAvoidance;

                        return target + avoidanceVector;
                    }

                    public Vector3D CalculateTargetInterceptionPoint(MyDetectedEntityInfo target, double timeSinceLastTargetDetection) //https://www.youtube.com/watch?v=MpUUsDDE1sI&list=LL&index=1 based on this fine video
                    {
                        Vector3D missileVelocity = missileData.linearVelocity / 2.35; //Magic number, not proud. Maybe I'll fix it in v2 release?
                        Vector3D relativeTargetPos = target.Position + target.Velocity * (float)timeSinceLastTargetDetection - missileData.position;
                        Vector3D relativeTargetVelocity = target.Velocity - missileVelocity;

                        double hitTime = SolveQuadraticEquation(relativeTargetVelocity.LengthSquared() - missileVelocity.LengthSquared(), 2 * Vector3D.Dot(relativeTargetPos, relativeTargetVelocity), relativeTargetPos.LengthSquared());
                        if (hitTime == 0)
                            return target.Position + target.Velocity * (float)relativeTargetPos.Length() / (float)missileVelocity.Length();

                        return target.Position + target.Velocity * (float)hitTime;
                    }

                    double SolveQuadraticEquation(double a, double b, double c)
                    {
                        double delta = b * b - 4 * a * c;
                        if (delta < 0 || a == 0)
                            return 0;
                        else if (delta == 0)
                            return -b / (2 * a);

                        return (-b - Math.Sqrt(delta)) / (2 * a);
                    }
                }

                public class OrientationModule
                {
                    //Flight settings
                    public float sideMomentumCorrectionFactor = 0.1F;
                    public float downMomentumCorrectionFactor = 0.1F;
                    public float upMomentumCorrectionFactor = 0.1F;
                    public float turnSpeedModifier = 1;

                    //In-game blocks
                    readonly List<IMyThrust> thrusters = new List<IMyThrust>();
                    readonly List<IMyGyro> gyros = new List<IMyGyro>();

                    //Logic
                    readonly Missile.MissileData missileData;
                    float maxThrust;

                    //Other
                    readonly MyGridProgram Program;

                    public OrientationModule(MyGridProgram Program, Missile.MissileData missileData, List<IMyThrust> thrusters, List<IMyGyro> gyros)
                    {
                        this.Program = Program;

                        this.thrusters = thrusters;
                        this.gyros = gyros;

                        this.missileData = missileData;
                    }

                    public void FlyTo(Vector3D target)
                    {
                        maxThrust = 0;
                        foreach (IMyThrust thruster in thrusters)
                            maxThrust += thruster.ThrustOverride;

                        Vector3D targetVector = target - missileData.position;
                        SetGyroscopesToVector(CalculateThrustDirection(targetVector));
                    }

                    void SetGyroscopesToVector(Vector3D vector)
                    {
                        Vector3D targetVector = Vector3D.Normalize(vector);

                        float targetYaw = turnSpeedModifier * (float)Math.Atan2(Vector3D.Dot(targetVector, missileData.worldMatrix.Right), Vector3D.Dot(targetVector, missileData.worldMatrix.Forward));
                        float targetPitch = turnSpeedModifier * (float)Math.Atan2(Vector3D.Dot(targetVector, missileData.worldMatrix.Down), Vector3D.Dot(targetVector, missileData.worldMatrix.Forward));

                        foreach (IMyGyro gyro in gyros)
                        {
                            gyro.Yaw = targetYaw;
                            gyro.Pitch = targetPitch;
                        }
                    }

                    Vector3D CalculateThrustDirection(Vector3D desiredDir)
                    {
                        double availableThrust = maxThrust;

                        Vector3D entireCorrection = CalculateShipCorrection(ref availableThrust, desiredDir, missileData.gravity);
                        if (desiredDir.IsZero() || availableThrust <= 0)
                            return entireCorrection;

                        Vector3D modifiedDesiredDir = Vector3D.Normalize(desiredDir) * availableThrust / missileData.mass;

                        return modifiedDesiredDir + entireCorrection;
                    }

                    Vector3D CalculateShipCorrection(ref double availableThrust, Vector3D desiredDir, Vector3D gravity)
                    {
                        Vector3D gravityCorrection = CalculateGravityCorrection(desiredDir, gravity);
                        Vector3D momentumCorrection = CalculateMomentumCorrection(desiredDir);

                        return CalculateEntireCorrection(ref availableThrust, gravityCorrection, momentumCorrection);
                    }

                    Vector3D CalculateGravityCorrection(Vector3D desiredDir, Vector3D gravity)
                    {
                        if (desiredDir.IsZero() || gravity.IsZero())
                            return -gravity;

                        return -Vector3D.ProjectOnPlane(ref gravity, ref desiredDir);
                    }

                    Vector3D CalculateMomentumCorrection(Vector3D desiredDir)
                    {
                        Vector3D momentum = missileData.linearVelocity;

                        if (desiredDir.IsZero())
                            return -missileData.linearVelocity;

                        return -Vector3D.ProjectOnPlane(ref momentum, ref desiredDir);
                    }

                    Vector3D CalculateEntireCorrection(ref double availableThrust, Vector3D gravityCorrection, Vector3D momentumCorrection)
                    {
                        if (gravityCorrection.IsZero() || momentumCorrection.IsZero())
                            return gravityCorrection + momentumCorrection * sideMomentumCorrectionFactor;

                        Vector3D gravityMomentumCorrection = Vector3D.ProjectOnVector(ref momentumCorrection, ref gravityCorrection);
                        Vector3D sideMomentumCorrection = (momentumCorrection - gravityMomentumCorrection) * sideMomentumCorrectionFactor;

                        if (Vector3D.Dot(gravityMomentumCorrection, gravityCorrection) < 0)
                            gravityMomentumCorrection *= upMomentumCorrectionFactor;
                        else
                            gravityMomentumCorrection *= downMomentumCorrectionFactor;

                        Vector3D entireGravityDirCorrection = gravityMomentumCorrection + gravityCorrection;

                        availableThrust = Math.Max(0, availableThrust - entireGravityDirCorrection.Length() * missileData.mass);

                        if (!sideMomentumCorrection.IsZero() && (float)sideMomentumCorrection.Length() * missileData.mass > availableThrust)
                            momentumCorrection *= availableThrust / ((float)sideMomentumCorrection.Length() * missileData.mass);

                        availableThrust = availableThrust - (float)sideMomentumCorrection.Length() * missileData.mass;

                        return entireGravityDirCorrection + sideMomentumCorrection;
                    }
                }
            }

            public class Radar
            {
                //Scan settings
                public int distance = 5000;
                public int scanConeAngle = 90;
                public int resolution = 50;

                //Tracking settings
                public MyDetectedEntityType? targetType = null;
                public MyRelationsBetweenPlayerAndBlock? targetRelationship = null;
                public double trackingOvershootFactor = 1.1;
                public double trackingAreaModifier = 1.25;

                //Accessible
                public MyDetectedEntityInfo detectedGrid { get; private set; }
                public double timeSinceLastDetection { get; private set; } = 0;
                public List<Vector3D> terrainPoints { get; private set; } = new List<Vector3D>();
                public List<Vector3D> obstaclePoints { get; private set; } = new List<Vector3D>();

                //In-game Blocks
                readonly List<IMyCameraBlock> cameras = new List<IMyCameraBlock>();

                //Logic
                int gridSize = 0;
                double rowOffset = 0;
                double columnOffset = 0;
                readonly Missile.MissileData missileData;
                bool terrainPointsSet = false;

                //Other
                readonly MyGridProgram Program;

                public Radar(MyGridProgram Program, Missile.MissileData missileData, List<IMyCameraBlock> cameras)
                {
                    this.Program = Program;
                    this.missileData = missileData;

                    this.cameras = cameras;
                    if (!cameras.Any()) return;

                    gridSize = (int)Math.Ceiling(Math.Sqrt(cameras.Count));
                    SortCameras(cameras);
                    foreach (var camera in cameras)
                        camera.EnableRaycast = true;
                }

                void SortCameras(List<IMyCameraBlock> cameras)
                {
                    Vector3 referencePos = cameras[0].GetPosition();
                    MatrixD transformationMatrix = missileData.worldMatrix;
                    cameras.Sort((a, b) =>
                    {
                        Vector3 dirA = a.GetPosition() - referencePos;
                        Vector3 bodyPosA = Vector3.TransformNormal(dirA, MatrixD.Transpose(transformationMatrix));

                        Vector3 dirB = b.GetPosition() - referencePos;
                        Vector3 bodyPosB = Vector3.TransformNormal(dirB, MatrixD.Transpose(transformationMatrix));

                        double deltaY = bodyPosB.Y - bodyPosA.Y;
                        if (Math.Round(deltaY, 1) != 0)
                            return deltaY > 0 ? 1 : -1;

                        return bodyPosB.X - bodyPosA.X > 0 ? 1 : -1;
                    });
                }

                public void Update(bool forceScan = false)
                {
                    if (!cameras.Any() || gridSize == 1) return;

                    timeSinceLastDetection += Program.Runtime.TimeSinceLastRun.TotalSeconds;

                    obstaclePoints.RemoveAll(obstacle => Vector3D.Dot(obstacle - cameras[0].GetPosition(), cameras[0].WorldMatrix.Forward) < 0);

                    if (detectedGrid.IsEmpty() || forceScan)
                        TryPerformCameraScan();
                    else
                        TryTrackTarget();
                }

                void TryPerformCameraScan()
                {
                    bool canCamerasScan = cameras[0].AvailableScanRange >= distance;

                    if (!canCamerasScan)
                        return;

                    double angularStep = MathHelper.ToDegrees(Math.Asin((double)resolution / (double)distance));
                    if (angularStep > scanConeAngle / gridSize)
                        angularStep = 0;

                    List<float[]> pitchYawPairs = GetPitchYawScanAngles(angularStep);

                    PerformCameraScan(pitchYawPairs);

                    columnOffset += angularStep;
                    if (columnOffset >= scanConeAngle / (gridSize - 1))
                    {
                        columnOffset = 0;
                        rowOffset += angularStep;
                    }
                    if (rowOffset >= scanConeAngle / (gridSize - 1))
                        rowOffset = 0;
                }

                List<float[]> GetPitchYawScanAngles(double angularStep)
                {
                    List<float[]> pitchYawPairs = new List<float[]>();
                    for (int row = 0; row < gridSize; row++)
                    {
                        for (int col = 0; col < gridSize; col++)
                        {
                            double stepSize = (scanConeAngle - Math.Sign(angularStep) * scanConeAngle / (gridSize - 1)) / (gridSize - 1);

                            double pitch = scanConeAngle / 2 - row * stepSize - rowOffset;
                            double yaw = scanConeAngle / 2 - col * stepSize - columnOffset;

                            pitchYawPairs.Add(new float[] { (float)pitch, (float)yaw });
                        }
                    }

                    return pitchYawPairs;
                }

                void PerformCameraScan(List<float[]> pitchYawPairs)
                {
                    for (int row = 0; row < gridSize; row++)
                    {
                        for (int col = 0; col < gridSize; col++)
                        {
                            int index = row * gridSize + col;

                            if (index >= cameras.Count) continue;

                            MyDetectedEntityInfo hitInfo = cameras[index].Raycast(distance, pitchYawPairs[index][0], pitchYawPairs[index][1]);

                            if (hitInfo.IsEmpty()) continue;

                            bool isTargetTypeValid = targetType == null || hitInfo.Type == targetType;
                            bool isTargetRelationshipValid = targetRelationship == null || hitInfo.Relationship == targetRelationship;

                            if (isTargetTypeValid && isTargetRelationshipValid)
                            {
                                if (detectedGrid.IsEmpty() || hitInfo.BoundingBox.Size.Length() > detectedGrid.BoundingBox.Size.Length())
                                    detectedGrid = hitInfo;

                                if (hitInfo.EntityId == detectedGrid.EntityId)
                                {
                                    if (!terrainPointsSet)
                                        terrainPoints.Add(hitInfo.HitPosition.Value);
                                    timeSinceLastDetection = 0;
                                }
                            }
                            else
                                obstaclePoints.Add(hitInfo.HitPosition.Value);
                        }
                    }

                    if (!terrainPointsSet && terrainPoints.Any())
                        terrainPointsSet = true;
                }

                void TryTrackTarget()
                {
                    Vector3D targetPredictedPos = detectedGrid.Position + detectedGrid.Velocity * (float)timeSinceLastDetection;
                    double trackingDistance = Vector3D.Distance(missileData.position, targetPredictedPos) * trackingOvershootFactor;

                    bool canCamerasTrack = cameras[0].AvailableScanRange >= distance;

                    if (!canCamerasTrack)
                        return;

                    List<float[]> pitchYawPairs = GetPitchYawTrackAngles(targetPredictedPos, trackingDistance);

                    PerformCameraTrack(pitchYawPairs, trackingDistance);
                }

                List<float[]> GetPitchYawTrackAngles(Vector3D targetPredictedPos, double trackingDistance)
                {
                    double targetSize = detectedGrid.BoundingBox.Size.Max();
                    double scanConeAngle = MathHelper.ToDegrees(Math.Asin(targetSize / trackingDistance)) * trackingAreaModifier;

                    List<float[]> pitchYawPairs = new List<float[]>();
                    for (int row = 0; row < gridSize; row++)
                    {
                        for (int col = 0; col < gridSize; col++)
                        {
                            int index = row * gridSize + col;

                            if (index >= cameras.Count) continue;

                            double yaw, pitch;
                            CalculateYawPitchFromDirection(targetPredictedPos - cameras[index].GetPosition(), missileData.worldMatrix, out yaw, out pitch);

                            pitch += scanConeAngle / 2 - row * scanConeAngle / (gridSize - 1);
                            yaw += scanConeAngle / 2 - col * scanConeAngle / (gridSize - 1);

                            pitchYawPairs.Add(new float[] { (float)MathHelper.Clamp(pitch, -45, 45), (float)MathHelper.Clamp(yaw, -45, 45) });
                        }
                    }

                    return pitchYawPairs;
                }

                void PerformCameraTrack(List<float[]> pitchYawPairs, double trackingDistance)
                {
                    for (int row = 0; row < gridSize; row++)
                    {
                        for (int col = 0; col < gridSize; col++)
                        {
                            int index = row * gridSize + col;

                            if (index >= cameras.Count) continue;

                            MyDetectedEntityInfo hitInfo = cameras[index].Raycast(trackingDistance, pitchYawPairs[index][0], pitchYawPairs[index][1]);

                            if (hitInfo.IsEmpty() || timeSinceLastDetection == 0) continue;

                            if (hitInfo.EntityId == detectedGrid.EntityId)
                            {
                                detectedGrid = hitInfo;
                                timeSinceLastDetection = 0;
                            }
                        }
                    }

                    if (timeSinceLastDetection > 0)
                        detectedGrid = new MyDetectedEntityInfo();
                }

                void CalculateYawPitchFromDirection(Vector3D dir, MatrixD worldMatrix, out double yaw, out double pitch)
                {
                    Vector3D localDir = Vector3D.TransformNormal(dir, MatrixD.Transpose(worldMatrix));
                    localDir = Vector3D.Normalize(localDir);

                    yaw = MathHelper.ToDegrees(Math.Atan2(localDir.X, -localDir.Z));
                    pitch = MathHelper.ToDegrees(Math.Atan2(localDir.Y, Math.Sqrt(Math.Pow(localDir.X, 2) + Math.Pow(-localDir.Z, 2))));
                }
            }
        }

        public class BombingComputer
        {
            //Bombardment settings
            public int distanceFromTargetToArm = 100;
            public double detonationTimeDelay = 0;
            public int minimumBombardmentAngle = 30;
            public double deadManSwitchTime = 10;
            public int carpetBombingLength = 0;

            //Logic settings
            public int maxSmallGridSpeed { set { ballisticComputer.maxSmallGridSpeed = value; } }

            //Accessible
            public double bombardmentDiameter { get { return ballisticComputer.bombardmentDiameter; } }
            public Vector3D bombardmentCenter { get { return ballisticComputer.bombardmentCenter; } }

            //Inner classes
            readonly BallisticComputer ballisticComputer;

            //In-game blocks
            readonly List<IMyShipMergeBlock> merges = new List<IMyShipMergeBlock>();
            readonly List<IMyWarhead> warheads = new List<IMyWarhead>();

            //Logic
            readonly Missile.MissileData missileData;
            readonly List<Bomblet> bomblets = new List<Bomblet>();
            double safetyTime = 3;

            //Other
            readonly MyGridProgram Program;

            class Bomblet
            {
                public IMyWarhead bomb;
                public float originalVelocity;
                public bool countdownStarted = false;
                public double detonationTime;

                public Bomblet(IMyWarhead bomb, float velocity, double detonationTime)
                {
                    this.bomb = bomb;
                    this.originalVelocity = velocity;
                    this.detonationTime = detonationTime;
                }

                public void Detonate()
                {
                    bomb.IsArmed = true;
                    bomb.Detonate();
                }
            }

            public BombingComputer(MyGridProgram Program, Missile.MissileData missileData, List<IMyShipMergeBlock> merges, List<IMyWarhead> warheads)
            {
                this.Program = Program;

                this.missileData = missileData;
                this.merges = merges;
                this.warheads = warheads;

                int[] mergeClusterData = SortMerges(merges);

                this.ballisticComputer = new BallisticComputer(Program, missileData, merges, mergeClusterData[0], mergeClusterData[1]);
            }

            int[] SortMerges(List<IMyShipMergeBlock> merges)
            {
                Vector3 referencePos = merges[0].GetPosition();
                MatrixD transformationMatrix = missileData.worldMatrix;
                HashSet<double> uniqueHeights = new HashSet<double>();

                merges.Sort((a, b) =>
                {
                    Vector3 dirA = a.GetPosition() - referencePos;
                    Vector3 bodyPosA = Vector3.TransformNormal(dirA, MatrixD.Transpose(transformationMatrix));

                    Vector3 dirB = b.GetPosition() - referencePos;
                    Vector3 bodyPosB = Vector3.TransformNormal(dirB, MatrixD.Transpose(transformationMatrix));

                    uniqueHeights.Add(Math.Round(bodyPosA.Z, 1));

                    double deltaZ = bodyPosB.Z - bodyPosA.Z;
                    if (Math.Round(deltaZ, 1) != 0)
                        return deltaZ < 0 ? 1 : -1;

                    double deltaY = bodyPosB.Y - bodyPosA.Y;
                    if (Math.Round(deltaY, 1) != 0)
                        return deltaY > 0 ? 1 : -1;

                    return bodyPosB.X - bodyPosA.X > 0 ? 1 : -1;
                });

                int clusterLevels = uniqueHeights.Count;
                int mergesPerLevel = merges.Count / clusterLevels;

                List<IMyShipMergeBlock> rearrangedMerges = new List<IMyShipMergeBlock>();
                for (int level = 0; level < clusterLevels; level++)
                {
                    for (int block = 0; block < mergesPerLevel / 2; block++)
                    {
                        rearrangedMerges.Add(merges[level * mergesPerLevel + block]);
                        rearrangedMerges.Add(merges[level * mergesPerLevel + mergesPerLevel - 1 - block]);
                    }
                }
                merges.Clear();
                merges.AddRange(rearrangedMerges);

                return new int[] { clusterLevels, mergesPerLevel };
            }

            public Vector3D ModifyTargetForCarpetBombing(Vector3D target)
            {
                if (missileData.gravity.IsZero() || carpetBombingLength == 0) return target;

                Vector3D targetDir = target - missileData.position;
                Vector3D carpetBombDir = Vector3D.ProjectOnPlane(ref targetDir, ref missileData.gravity).Normalized();
                Vector3D carpetBombStart = target - carpetBombDir * (carpetBombingLength / 2);

                double totalMerges = ballisticComputer.clusterLevels * ballisticComputer.mergesPerLevel;
                double deployedBombRatio = (totalMerges - merges.Count) / totalMerges;

                return carpetBombStart + carpetBombDir * carpetBombingLength * deployedBombRatio;
            }

            public Vector3D AddTargetOvershootCorrection(Vector3D target, int overshootCorrection)
            {
                if (missileData.gravity.IsZero()) return target;

                Vector3D targetDir = target - missileData.position;
                Vector3D overshootCorrectionVec = Vector3D.ProjectOnPlane(ref targetDir, ref missileData.gravity);

                if(overshootCorrectionVec.IsZero()) return target;

                return target - overshootCorrectionVec.Normalized() * overshootCorrection;
            }

            public void UpdateBombardmentData(Vector3D target)
            {
                ballisticComputer.UpdateBombardmentData(target);
            }

            public Vector3D CalculateBombardmentPath(Vector3D target, double maxAngleToTarget = 30)
            {
                Vector3D gravity = missileData.gravity;
                Vector3D bombardmentCorrection = target - ballisticComputer.bombardmentCenter;
                double bombardmentError = bombardmentCorrection.Length();

                if (gravity.IsZero())
                    return target + bombardmentCorrection.Normalized() * Math.Sqrt(bombardmentError) * 4;

                Vector3D targetDir = target - missileData.position;
                double attackAngle = Vector3D.Angle(gravity, targetDir);

                if (minimumBombardmentAngle > 0 && attackAngle > MathHelper.ToRadians(90 - minimumBombardmentAngle))
                    return target + -Vector3D.ProjectOnPlane(ref gravity, ref targetDir).Normalized() * Math.Tan(maxAngleToTarget) * targetDir.Length();

                bombardmentCorrection = Vector3D.ProjectOnPlane(ref bombardmentCorrection, ref targetDir);
                double correctionModifier = MathHelper.Lerp(13.01, 128, gravity.Length() / 9.81) * Math.Sqrt(bombardmentError) / Math.Max(1, 2000 / targetDir.Length()); //The lerp for Moon will output 32, for Earth 128
                Vector3D targetHeading = new PlaneD(target, targetDir).Intersection(ref missileData.position, ref missileData.linearVelocity) + bombardmentCorrection.Normalized() * correctionModifier;
                Vector3D targetRelativeHeading = targetHeading - target;

                double maxAngle = attackAngle < MathHelper.ToRadians(maxAngleToTarget) ? attackAngle : MathHelper.ToRadians(maxAngleToTarget);
                if (Math.Atan(targetRelativeHeading.Length() / targetDir.Length()) > maxAngle)
                    return target + targetRelativeHeading.Normalized() * Math.Tan(maxAngle) * targetDir.Length();

                return targetHeading;
            }

            public float CalculateBombardmentRoll(MyDetectedEntityInfo target, double explosionRadius)
            {
                double optimalBombardmentDiameter = (Math.Sqrt(warheads.Count) - 1) * Math.Sqrt(Math.Pow(explosionRadius, 2) / 2);
                if (target.IsEmpty())
                    return (float)(optimalBombardmentDiameter - ballisticComputer.bombardmentDiameter);

                Vector3D targetDiagonal = target.BoundingBox.Max - target.BoundingBox.Min;
                Vector3D localDiagonal = Vector3D.TransformNormal(targetDiagonal, MatrixD.Transpose(missileData.worldMatrix));
                double smallestTargetProjectionDimension = Math.Min(Math.Abs(localDiagonal.X), Math.Abs(localDiagonal.Y));

                return (float)(Math.Min(optimalBombardmentDiameter, smallestTargetProjectionDimension) - ballisticComputer.bombardmentDiameter);
            }

            public void SplitNextMerges(int mergesToSplit = 1)
            {
                int count = Math.Min(mergesToSplit, merges.Count);
                for (int i = 0; i < count; i++)
                    merges[i].Enabled = false;
                merges.RemoveRange(0, count);
            }

            public void ManageBombExplosions(Vector3D target)
            {
                if (!warheads.Any()) return;

                if (safetyTime > 0)
                {
                    safetyTime -= Program.Runtime.TimeSinceLastRun.TotalSeconds;
                    return;
                }

                if (!bomblets.Any())
                {
                    for (int i = 0; i < warheads.Count; i++)
                        bomblets.Add(new Bomblet(warheads[i], warheads[i].CubeGrid.Speed, (i * 0.25) % (detonationTimeDelay + 0.25)));
                    return;
                }

                deadManSwitchTime -= Program.Runtime.TimeSinceLastRun.TotalSeconds;

                foreach (Bomblet bomblet in bomblets)
                {
                    if (bomblet.countdownStarted)
                        bomblet.detonationTime -= Program.Runtime.TimeSinceLastRun.TotalSeconds;
                    else if (bomblet.originalVelocity - bomblet.bomb.CubeGrid.Speed > 1)
                        bomblet.countdownStarted = true;
                }

                bomblets.RemoveAll(bomblet =>
                {
                    if (deadManSwitchTime < 0 || (bomblet.countdownStarted && bomblet.detonationTime <= 0))
                    {
                        if(bomblet.bomb != null)
                            bomblet.Detonate();
                        return true;
                    }

                    return false;
                });

                if (detonationTimeDelay > 0 || warheads.Any(warhead => Vector3D.Distance(warhead.GetPosition(), target) > distanceFromTargetToArm))
                    return;

                foreach (Bomblet bomblet in bomblets)
                    bomblet.bomb.IsArmed = true;
            }

            public class BallisticComputer
            {
                //Ballistic settings
                public int maxSmallGridSpeed = 100;

                //Accessible
                public double bombardmentDiameter;
                public Vector3D bombardmentCenter;
                public int clusterLevels;
                public int mergesPerLevel;

                //In-game blocks
                readonly List<IMyShipMergeBlock> merges = new List<IMyShipMergeBlock>();

                //Logic
                readonly Missile.MissileData missileData;

                //Other
                readonly MyGridProgram Program;

                public BallisticComputer(MyGridProgram Program, Missile.MissileData missileData, List<IMyShipMergeBlock> merges, int clusterLevels, int mergesPerLevel)
                {
                    this.Program = Program;
                    this.missileData = missileData;

                    this.merges = merges;

                    this.clusterLevels = clusterLevels;
                    this.mergesPerLevel = mergesPerLevel;
                }

                public void UpdateBombardmentData(Vector3D target)
                {
                    if (merges.Count < mergesPerLevel) return;

                    int existingWholeLayers = merges.Count / mergesPerLevel;
                    int firstCornerMerge = merges.Count - existingWholeLayers * mergesPerLevel;

                    int mergesPerSide = 2 * (int)Math.Ceiling(Math.Sqrt(mergesPerLevel / 2));
                    IMyShipMergeBlock[] cornerMerges = { merges[firstCornerMerge], merges[firstCornerMerge + 1], merges[firstCornerMerge + mergesPerSide - 2], merges[firstCornerMerge + mergesPerSide - 1] };
                    List<Vector3D> hits = new List<Vector3D>();

                    for (int i = 0; i < cornerMerges.Length; i++)
                        hits.Add(CalculateMergeBlockEndPosition(cornerMerges[i], target));

                    bombardmentDiameter = Math.Max(Vector3D.Distance(hits[0], hits[1]), Vector3D.Distance(hits[2], hits[3]));
                    bombardmentCenter = (hits[0] + hits[1] + hits[2] + hits[3]) / 4;
                }

                Vector3D CalculateMergeBlockEndPosition(IMyShipMergeBlock merge, Vector3D terminationPoint, float maxSimulationTimeSeconds = 60)
                {
                    Vector3D gravity = missileData.gravity;
                    bool isMissileAboveTarget = Vector3D.Dot(terminationPoint - missileData.position, gravity) > 0;

                    Vector3D startingTargetVector = terminationPoint - merge.GetPosition();
                    Vector3D startingGravityProjectedTargetVector = Vector3D.ProjectOnVector(ref startingTargetVector, ref gravity);

                    Vector3D velocity = Vector3D.ClampToSphere(missileData.linearVelocity + CalculateMergeBlockCentrifugalForce(merge) + gravity / 60, maxSmallGridSpeed);
                    Vector3D position = merge.GetPosition() + velocity / 60;

                    for (float i = 0; i < maxSimulationTimeSeconds * 60; i += 1)
                    {
                        Vector3D targetVector = terminationPoint - position;
                        if (Vector3D.Dot(targetVector, startingTargetVector) < 0)
                            break;

                        Vector3D gravityProjectedTargetVector = Vector3D.ProjectOnVector(ref targetVector, ref gravity);
                        if (isMissileAboveTarget && Vector3D.Dot(gravityProjectedTargetVector, startingGravityProjectedTargetVector) < 0)
                            break;

                        velocity = Vector3D.ClampToSphere(velocity + gravity / 60, maxSmallGridSpeed);
                        position += velocity / 60;
                    }

                    return position;
                }

                Vector3D CalculateMergeBlockCentrifugalForce(IMyShipMergeBlock merge)
                {
                    int mergeLevel = merges.IndexOf(merge) / mergesPerLevel;

                    Vector3D missileCenterAtMergeLevel = (merges[mergeLevel * mergesPerLevel].GetPosition() + merges[mergeLevel * mergesPerLevel + 1].GetPosition()) / 2;
                    Vector3D radius = merge.GetPosition() - missileCenterAtMergeLevel;

                    return Vector3D.Cross(missileData.angularVelocity, radius);
                }
            }
        }
    }
}
