using Sandbox.Game.Entities;
using Sandbox.Game.Entities.Blocks;
using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Diagnostics;
using System.Linq;
using System.Runtime.Remoting.Messaging;
using System.Security.Policy;
using System.Text;
using VRage;
using VRage.Collections;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.GUI.TextPanel;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ObjectBuilders.Definitions;
using VRage.Meta;
using VRageMath;
using VRageRender;
using static IngameScript.Program.Missile;


namespace IngameScript
{
    partial class Program : MyGridProgram
    {
        int timeForGuidanceActivation = 1;
        Vector3 gpsCoordinates = new Vector3(10209.14F, 22497.27F, -55378.12F);

        DebugAPI Debug;
        public Program()
        {
            Debug = new DebugAPI(this);

            panel = GridTerminalSystem.GetBlockWithName("LCD") as IMyTextPanel;
            List<IMyBlockGroup> blockGroups = new List<IMyBlockGroup>();
            GridTerminalSystem.GetBlockGroups(blockGroups);
            launcherConnectors = blockGroups.OfType<IMyShipConnector>().ToList();
        }

        List<IMyShipConnector> launcherConnectors = new List<IMyShipConnector>();
        IMyTextPanel panel;

        Missile missile;

        int maxAcceptableHitError = 20;

        //In-Gravity Settings
        int desiredCruiseAltitude = 2000;
        int desiredSplitDistance = 1000;
        int desiredAttackAngleInGravity = 45;

        bool isAngleReached = false;
        bool isAttackReached = false;

        PIDController bombingPID = new PIDController(3,0.5,0.4);

        public void Main(string argument, UpdateType updateSource)
        {
            Debug.RemoveAll(); 

            if (missile != null)
            {
                missile.MandatoryMissileUpdate(Runtime.UpdateFrequency);

                IMyShipController controller = missile.TryGetWorkingRemote();
                Vector3 gravity = controller.GetNaturalGravity();
                Vector3 enemyVector = gpsCoordinates - missile.CubeGrid.GetPosition();

                Vector3 modifiedTargetPoint = gpsCoordinates;

                if (missile.elapsedTime > timeForGuidanceActivation)
                {
                    if (gravity.Length() > 0)
                    {
                        if (!isAngleReached)
                        {
                            int angle = 90 - (int)MathHelper.ToDegrees(Vector3.Angle(gravity, enemyVector));
                            if(angle == desiredAttackAngleInGravity)
                                isAngleReached = true;
                            
                            if (angle < desiredAttackAngleInGravity)
                                modifiedTargetPoint -= controller.GetNaturalGravity() * (desiredCruiseAltitude / controller.GetNaturalGravity().Length());
                            else
                                modifiedTargetPoint += controller.GetNaturalGravity() * (desiredCruiseAltitude / controller.GetNaturalGravity().Length());
                        }
                        else if(!isAttackReached && enemyVector.Length() <= desiredSplitDistance + 1500)
                        {
                            //Hit distance, hit angle, hit side

                            Runtime.UpdateFrequency |= UpdateFrequency.Update1;
                            Runtime.UpdateFrequency &= ~UpdateFrequency.Update10;
                            Vector3[] vectors = GetSideMostClusterHits(missile.clusterMerges, gpsCoordinates);
                            Vector3 avarage = (vectors[0] + vectors[1] + vectors[2] + vectors[3]) / 4;

                            Debug.DrawLine(missile.CubeGrid.GetPosition(), vectors[0], Color.Black);
                            Debug.DrawLine(missile.CubeGrid.GetPosition(), vectors[1], Color.Gray);
                            Debug.DrawLine(missile.CubeGrid.GetPosition(), vectors[2], Color.LightGray);
                            Debug.DrawLine(missile.CubeGrid.GetPosition(), vectors[3], Color.White);

                            Vector3 missileToTarget = missile.CubeGrid.GetPosition() - gpsCoordinates;
                            Vector3 hitToTarget = avarage - gpsCoordinates;
                            float hitError = (gpsCoordinates - avarage).Length() * Math.Sign(Vector3.Dot(Vector3.ProjectOnPlane(ref missileToTarget, ref gravity), Vector3.ProjectOnPlane(ref hitToTarget, ref gravity)));
                            if (hitError < 0)
                                hitError /= 2;

                            float pidModifier = (float)bombingPID.Calculate(hitError, Runtime.LastRunTimeMs);
                            float angleOfAttackModifier = (float)(gravity.Length() == 0 ? 1 : Math.Log10((int)MathHelper.ToDegrees(Vector3.Angle(gravity, enemyVector))));
                            float distanceModifier = (float)Math.Sqrt(200 / missileToTarget.Length());
                            modifiedTargetPoint += -controller.GetNaturalGravity().Normalized() * pidModifier * angleOfAttackModifier * distanceModifier;

                            Debug.DrawPoint(modifiedTargetPoint, Color.Pink, 5);
                            Debug.DrawPoint(avarage, Color.Purple, 5);

                            if (enemyVector.Length() <= desiredSplitDistance && Math.Abs(hitError) <= maxAcceptableHitError)
                            {
                                foreach (IMyGyro gyro in missile.gyros)
                                    gyro.Roll = -60;
                                isAttackReached = true;
                            } 
                        }
                        else if(isAttackReached)
                        {
                            Vector3[] vectors = GetSideMostClusterHits(missile.clusterMerges, gpsCoordinates);
                            Vector3 avarage = (vectors[0] + vectors[1] + vectors[2] + vectors[3]) / 4;
                            float spreadDiameter = 0.5F * (vectors[0] - vectors[1]).Length() + 0.5F * (vectors[2] - vectors[3]).Length();

                            Echo(spreadDiameter.ToString());
                            if(spreadDiameter > 15)
                                missile.SplitNextLayer();
                        }

                        if(!isAttackReached)
                            missile.flightComputer.FlyTo(modifiedTargetPoint);
                    }
                    else
                    {

                    }
                }

                /*
                 if (!missile.targetingSystem.detectedEnemy.IsEmpty())
                    {
                        Vector3 interceptionPoint = missile.CubeGrid.GetPosition() + missile.targetingSystem.missileGuidance.ProportionalNavigation(missile.targetingSystem.detectedEnemy);
                        if (interceptionPoint.Length() == 0)
                            interceptionPoint = missile.targetingSystem.detectedEnemy.Position;
                        Debug.DrawLine(missile.CubeGrid.GetPosition(), interceptionPoint, Color.Red);
                    }
                    else
                        missile.flightComputer.FlyTo(targetPoint);
                  
                if (missile.cubeGridUpdated)
                {
                    Vector3[] vectors = GetSideMostClusterHits(missile.clusterMerges, gpsCoordinates);
                    Vector3 avarage = (vectors[0] + vectors[1] + vectors[2] + vectors[3]) / 4;
                    Vector3 hitToTargetVector = gpsCoordinates - avarage;

                    IMyShipController controller = missile.TryGetWorkingRemote();
                    if(controller != null && controller.GetNaturalGravity().Length() > 0)
                        if (Vector3.Dot(gpsCoordinates - missile.CubeGrid.GetPosition(), hitToTargetVector) > 0)
                            targetPoint -= controller.GetNaturalGravity() * (desiredCruiseAltitude / controller.GetNaturalGravity().Length());
                    Debug.DrawPoint(avarage, Color.Purple, 3, 1);
                }

                /*
                if (((updateSource & UpdateType.Update1) != 0) && (gpsCoordinates - missile.CubeGrid.GetPosition()).Length() < 1250)
                {
                    missile.SplitNextLayer();
                }
                else if ((gpsCoordinates - missile.CubeGrid.GetPosition()).Length() < 1500)
                {
                    foreach (IMyGyro gyro in missile.gyros)
                        gyro.Roll = -60;
                    Runtime.UpdateFrequency |= UpdateFrequency.Update1;
                    Runtime.UpdateFrequency &= ~UpdateFrequency.Update10;
                    Debug.RemoveAll();
                }*/

                /*
                if((gpsCoordinates - missile.CubeGrid.GetPosition()).Length() < 1000)
                {
                    foreach (IMyWarhead warhead in missile.warheads)
                        warhead.IsArmed = true;
                    foreach (IMyGyro gyro in missile.gyros)
                        gyro.Roll = -60;

                    Runtime.UpdateFrequency |= UpdateFrequency.Update1;
                    Runtime.UpdateFrequency &= ~UpdateFrequency.Update10;
                }

                if (((updateSource & UpdateType.Update1) != 0) && (gpsCoordinates - missile.CubeGrid.GetPosition()).Length() < 750)
                    missile.SplitNextLayer(); */
            }

            if (argument == "launch")
                LaunchMissile();
        }

        public Vector3[] GetSideMostClusterHits(List<IMyShipMergeBlock> clusterMerges, Vector3 enemyPos)
        {
            IMyShipController controller = missile.TryGetWorkingRemote();
            Vector3 gravity = controller.GetNaturalGravity();

            Vector3 hitPosOne = GetEntityEndPosition(missile.bombCluster.bombs[0].merge, controller.GetShipVelocities().AngularVelocity, gravity, gravity, enemyPos);
            Vector3 hitPosTwo = GetEntityEndPosition(missile.bombCluster.bombs[1].merge, controller.GetShipVelocities().AngularVelocity, gravity, gravity, enemyPos);
            Vector3 hitPosThree = GetEntityEndPosition(missile.bombCluster.bombs[missile.bombCluster.clusterWidth - 1].merge, controller.GetShipVelocities().AngularVelocity, gravity, gravity, enemyPos);
            Vector3 hitPosFour = GetEntityEndPosition(missile.bombCluster.bombs[missile.bombCluster.clusterWidth - 2].merge, controller.GetShipVelocities().AngularVelocity, gravity, gravity, enemyPos);

            return new Vector3[] { hitPosOne, hitPosTwo, hitPosThree, hitPosFour };
        }

        //--------------------------------------------------------------------------------------------------------------------
        //TRAJECTORY PREDICTION (ITERATIVE SIMULATION + RUNGE-KUTTA + CURSED MODIFIERS) START

        public Vector3 GetEntityEndPosition(IMyTerminalBlock entity, Vector3 angularVelocity, Vector3 acceleration, Vector3 gravity, Vector3 target, float maxSimTime = 60)
        {
            Vector3D velocity = Vector3.ClampToSphere(entity.CubeGrid.LinearVelocity + AngularToLinearVelocity(entity, angularVelocity), 100 * 5 / 4);
            Vector3D position = entity.GetPosition() + velocity * 0.5F;

            Debug.DrawLine(position, position + AngularToLinearVelocity(entity, angularVelocity), Color.Orange);

            Vector3 targetTerminationNormal = target - position;
            Vector3 gravityTerminationNormal = gravity.Length() == 0 ? gravity : Vector3.ProjectOnVector(ref targetTerminationNormal, ref gravity);

            Plane gravityTerminationPlane = new Plane(position + gravityTerminationNormal, gravity);
            Plane targetTerminationPlane = new Plane(position + targetTerminationNormal, targetTerminationNormal);

            Debug.DrawLine(position, position + targetTerminationNormal, Color.LightGreen);
            Debug.DrawLine(position, position + gravityTerminationNormal, Color.LightGreen);
            Debug.DrawLine(position + gravityTerminationNormal, position + targetTerminationNormal, Color.LightGreen);

            for (float i = 0; i < maxSimTime && i < 60; i += 1F)
            {
                Vector3 k1v = velocity + acceleration;
                Vector3 k2v = velocity + k1v / 2 + acceleration * 1.5F;
                Vector3 k3v = velocity + k2v / 2 + acceleration * 1.5F;
                Vector3 k4v = velocity + k3v + acceleration * 2;

                velocity = Vector3.ClampToSphere((k1v + 2 * k2v + 2 * k3v + k4v) / 6, 100 * 5 / 4);
                position += velocity;

                Debug.DrawLine(position - velocity, position, Color.Red);

                Vector3D prevPosition = position - velocity;

                if(gravity.Length() > 0) {
                    if(Vector3.Dot(gravityTerminationNormal, target - position) < 0)
                        return gravityTerminationPlane.Intersection(ref prevPosition, ref velocity);
                }
                else if(Vector3.Dot(targetTerminationNormal, target - position) < 0)
                    return targetTerminationPlane.Intersection(ref prevPosition, ref velocity);
            }

            return position;
        }

        //TRAJECTORY PREDICTION (ITERATIVE SIMULATION + RUNGE-KUTTA + CURSED MODIFIERS) END
        //--------------------------------------------------------------------------------------------------------------------

        public Vector3 AngularToLinearVelocity(IMyTerminalBlock entity, Vector3 angularVelocity)
        {
            IMyCubeGrid CubeGrid = entity.CubeGrid;
            Vector3 volumeVector = CubeGrid.Max - CubeGrid.Min;

            Vector3 entityAbstractPosition, gridCenterAtEntityHeight;
            if (volumeVector.X > volumeVector.Y && volumeVector.X > volumeVector.Z)
            {
                entityAbstractPosition = new Vector3(entity.Position.Y, entity.Position.X, entity.Position.Z);
                gridCenterAtEntityHeight = new Vector3(CubeGrid.Min.Y + volumeVector.Y / 2, entity.Position.X, CubeGrid.Min.Z + volumeVector.Z / 2);
            }
            else if (volumeVector.Z > volumeVector.Y && volumeVector.Z > volumeVector.X)
            {
                entityAbstractPosition = new Vector3(entity.Position.X, entity.Position.Z, entity.Position.Y);
                gridCenterAtEntityHeight = new Vector3(CubeGrid.Min.X + volumeVector.X / 2, entity.Position.Z, CubeGrid.Min.Y + volumeVector.Y / 2);
            }
            else
            {
                entityAbstractPosition = new Vector3(entity.Position.X, entity.Position.Y, entity.Position.Z);
                gridCenterAtEntityHeight = new Vector3(CubeGrid.Min.X + volumeVector.X / 2, entity.Position.Y, CubeGrid.Min.Z + volumeVector.Z / 2);
            }

            Vector3 radius = Vector3.TransformNormal(entityAbstractPosition - gridCenterAtEntityHeight, missile.WorldMatrix);

            return Vector3.ClampToSphere(Vector3.Cross(angularVelocity, radius), 1200 / CubeGrid.LinearVelocity.Length());
        }

        public void LaunchMissile()
        {
            InitializeMissile();
            missile.Disconnect();
            Runtime.UpdateFrequency |= UpdateFrequency.Update10;
        }

        public void InitializeMissile()
        {
            List<IMyTerminalBlock> allBlocks = new List<IMyTerminalBlock>();
            GridTerminalSystem.GetBlocks(allBlocks);

            List<IMyShipMergeBlock> missileDockingMerges = new List<IMyShipMergeBlock>();
            IMyBlockGroup group = GridTerminalSystem.GetBlockGroupWithName("MissileDockingMerges");
            group.GetBlocksOfType(missileDockingMerges);

            List<IMyTerminalBlock> missileBlocks = allBlocks.Where(block => block.CubeGrid == Me.CubeGrid).ToList();
            missile = new Missile(
                                  missileDockingMerges,
                                  missileBlocks.OfType<IMyShipMergeBlock>().Except(missileDockingMerges).ToList(),
                                  missileBlocks.OfType<IMyShipConnector>().Except(launcherConnectors).ToList(),
                                  missileBlocks.OfType<IMyRemoteControl>().ToList(),
                                  missileBlocks.OfType<IMyTurretControlBlock>().ToList(),
                                  missileBlocks.OfType<IMyBatteryBlock>().ToList(),
                                  missileBlocks.OfType<IMyGasTank>().ToList(),
                                  missileBlocks.OfType<IMyThrust>().ToList(),
                                  missileBlocks.OfType<IMyGyro>().ToList(),
                                  missileBlocks.OfType<IMyCameraBlock>().ToList(),
                                  missileBlocks.OfType<IMyWarhead>().ToList()
                                 );

            List<IMyMotorStator> motors = allBlocks.OfType<IMyMotorStator>().Where(motor => motor.TopGrid == Me.CubeGrid).ToList();
            for(int i = 0; i < missile.turretControls.Count; i++)
            {
                missile.turretControls[i].ElevationRotor = motors[0];
                missile.turretControls[i].AzimuthRotor = motors[0];
                missile.turretControls[i].Camera = missile.cameras[(missile.cameras.Count - 1) / (i + 1)];
                missile.turretControls[i].Range = 600;
            }
        }

        public class Missile
        {
            public Missile (List<IMyShipMergeBlock> dockingMerges, List<IMyShipMergeBlock> clusterMerges, List<IMyShipConnector> connectors, List<IMyRemoteControl> remotes, List<IMyTurretControlBlock> turretControls, List<IMyBatteryBlock> batteries, List<IMyGasTank> tanks, List<IMyThrust> thrusters, List<IMyGyro> gyros, List<IMyCameraBlock> cameras, List<IMyWarhead> warheads)
            {
                flightComputer = new FlightComputer(this);
                targetingSystem = new TargetingSystem(this);

                WorldMatrix = remotes[0].WorldMatrix;
                CubeGrid = dockingMerges[0].CubeGrid;

                this.dockingMerges = dockingMerges;
                this.clusterMerges = clusterMerges;
                this.connectors = connectors;
                this.remotes = remotes;
                this.turretControls = turretControls;
                this.batteries = batteries;
                this.tanks = tanks;
                this.thrusters = thrusters;
                this.gyros = gyros;
                this.cameras = cameras;
                this.warheads = warheads;
            }

            public FlightComputer flightComputer;
            public TargetingSystem targetingSystem;

            public struct Bomb
            {
                public IMyShipMergeBlock merge;
                public IMyWarhead warhead;
                public Bomb(IMyShipMergeBlock merge, IMyWarhead warhead)
                {
                    this.merge = merge;
                    this.warhead = warhead;
                }
            }

            public struct BombCluster
            {
                public int clusterWidth;
                public List<Bomb> bombs;
            }

            public BombCluster bombCluster;
            public List<IMyShipMergeBlock> dockingMerges = new List<IMyShipMergeBlock>();
            public List<IMyShipMergeBlock> clusterMerges = new List<IMyShipMergeBlock>();
            public List<IMyShipConnector> connectors = new List<IMyShipConnector>();
            public List<IMyRemoteControl> remotes = new List<IMyRemoteControl>();
            public List<IMyTurretControlBlock> turretControls = new List<IMyTurretControlBlock>();
            public List<IMyBatteryBlock> batteries = new List<IMyBatteryBlock>();
            public List<IMyGasTank> tanks = new List<IMyGasTank>();
            public List<IMyThrust> thrusters = new List<IMyThrust>();
            public List<IMyGyro> gyros = new List<IMyGyro>();
            public List<IMyCameraBlock> cameras = new List<IMyCameraBlock>();
            public List<IMyWarhead> warheads = new List<IMyWarhead>();

            public Matrix WorldMatrix;
            public IMyCubeGrid CubeGrid;
            public bool cubeGridUpdated = false;

            public float elapsedTime = 0;        

            public void Disconnect()
            {
                foreach (IMyShipConnector connector in connectors)
                    connector.Disconnect();

                foreach (IMyShipMergeBlock merge in dockingMerges)
                    merge.Enabled = false;

                foreach (IMyThrust thruster in thrusters)
                {
                    thruster.ThrustOverride = thruster.MaxThrust;
                    thruster.Enabled = true;
                }

                foreach (IMyGyro gyro in gyros)
                    gyro.GyroOverride = true;

                foreach (IMyGasTank tank in tanks)
                    tank.Stockpile = false;

                foreach (IMyBatteryBlock battery in batteries)
                    battery.ChargeMode = ChargeMode.Auto;

                foreach (IMyTurretControlBlock turretControl in turretControls)
                    turretControl.AIEnabled = true;
            }

            public void MandatoryMissileUpdate(UpdateFrequency updateType)
            {
                if (!cubeGridUpdated)
                    CheckIfCubeGridUpdated();
                UpdateMissileInfo(updateType);
                if (!IsSteerable())
                    ;//emergency boom function
            }

            public void CheckIfCubeGridUpdated()
            {
                bool hasUpdated = false;
                if (CubeGrid.Min.X == dockingMerges[0].Position.X || CubeGrid.Min.Y == dockingMerges[0].Position.Y || CubeGrid.Min.Z == dockingMerges[0].Position.Z)
                    hasUpdated = true;

                if(hasUpdated)
                {
                    List<IMyShipMergeBlock> launcherMerges = clusterMerges.Where(merge => merge.CubeGrid != CubeGrid).ToList();
                    clusterMerges.RemoveAll(merge => launcherMerges.Contains(merge));
                    InitializeBombClusterStruct(clusterMerges);
                    cubeGridUpdated = true;
                }
            }

            private void UpdateMissileInfo(UpdateFrequency updateType)
            {
                if (updateType != 0)
                {
                    int frequencyBit = (int)Math.Log((int)updateType, 2);
                    elapsedTime += 1.66F * (float)Math.Pow(10, frequencyBit - 2);
                }

                flightComputer.UpdateFlightInfo();
                targetingSystem.UpdateEnemyInfo();
            }

            public IMyRemoteControl TryGetWorkingRemote()
            {
                return remotes.FirstOrDefault(remote => remote != null && remote.IsWorking);
            }

            private int CameraCount()
            {
                return cameras.Count(camera => camera != null && camera.IsWorking);
            }

            private bool IsSteerable()
            {
                float storedPower = 0;
                foreach (IMyBatteryBlock battery in batteries)
                    if (battery != null && battery.IsWorking)
                        storedPower += battery.CurrentStoredPower;

                if (storedPower == 0)
                    return false;

                double storedFuel = 0;
                foreach (IMyGasTank tank in tanks)
                    if (tank != null && tank.IsWorking)
                        storedFuel += tank.FilledRatio;

                if (storedFuel == 0)
                    return false;

                int workingGyros = 0;
                foreach (IMyGyro gyro in gyros)
                    if (gyro != null && gyro.IsWorking)
                        workingGyros++;

                if(workingGyros == 0)
                    return false;

                return flightComputer.missileMass * flightComputer.maxThrust > flightComputer.lastKnownGravity.Length();
            }

            public void SplitNextLayer()
            {
                clusterMerges[0].Enabled = false;
                clusterMerges[1].Enabled = false;
                clusterMerges.RemoveAt(1);
                clusterMerges.RemoveAt(0);

                if (clusterMerges.Count <= 1)
                    foreach (IMyWarhead warhead in warheads)
                        warhead.IsArmed = true;
            }

            //--------------------------------------------------------------------------------------------------------------------
            //CURSED SPLIT SEQUENCE SORT ALGORITHM START (USING DYNAMICALLY DEFINED STRUCT LOL)

            private void InitializeBombClusterStruct(List<IMyShipMergeBlock> merges)
            {
                Vector3I volumeVector = CubeGrid.Max - CubeGrid.Min;
                int[] warheadMergeOffset = { 0, 0, 0 };

                var abstractMerges = new[] { new { listPos = 1, X = 0, Y = 0, Z = 0 } }.ToList();
                abstractMerges.Clear();
                if (volumeVector.X > volumeVector.Y && volumeVector.X > volumeVector.Z)
                {
                    warheadMergeOffset[0] = CubeGrid.GetCubeBlock(CubeGrid.Max).FatBlock as IMyCameraBlock != null ? 1 : -1;
                    bombCluster.clusterWidth = volumeVector.Y + 1;
                    for (int i = 0; i < merges.Count; i++)
                        abstractMerges.Add(new { listPos = i, X = merges[i].Position.Y - CubeGrid.Min.Y, Y = merges[i].Position.X - CubeGrid.Min.X, Z = merges[i].Position.Z - CubeGrid.Min.Z });
                }
                else if (volumeVector.Z > volumeVector.Y && volumeVector.Z > volumeVector.X)
                {
                    warheadMergeOffset[2] = CubeGrid.GetCubeBlock(CubeGrid.Max).FatBlock as IMyCameraBlock != null ? 1 : -1;
                    bombCluster.clusterWidth = volumeVector.X + 1;
                    for (int i = 0; i < merges.Count; i++)
                        abstractMerges.Add(new { listPos = i, X = merges[i].Position.X - CubeGrid.Min.X, Y = merges[i].Position.Z - CubeGrid.Min.Z, Z = merges[i].Position.Y - CubeGrid.Min.Y });
                }
                else
                {
                    warheadMergeOffset[1] = CubeGrid.GetCubeBlock(CubeGrid.Max).FatBlock as IMyCameraBlock != null ? 1 : -1;
                    bombCluster.clusterWidth = volumeVector.X + 1;
                    for (int i = 0; i < merges.Count; i++)
                        abstractMerges.Add(new { listPos = i, X = merges[i].Position.X - CubeGrid.Min.X, Y = merges[i].Position.Y - CubeGrid.Min.Y, Z = merges[i].Position.Z - CubeGrid.Min.Z });
                }

                List<int> abstractMergesHeights = new List<int>();
                foreach (var abstractMerge in abstractMerges)
                    abstractMergesHeights.Add(abstractMerge.Y);

                int minAbstractMergeHeight = abstractMergesHeights.Min();

                for (int i = abstractMerges.Count - 1; i >= 0; i--)
                {
                    if ((abstractMerges[i].Y - minAbstractMergeHeight) % 3 == 0)
                        abstractMerges.RemoveAt(i);
                }

                var rightMerges = new[] { new { listPos = 1, X = 0, Y = 0, Z = 0 } }.ToList();
                rightMerges.Clear();
                var leftMerges = new[] { new { listPos = 1, X = 0, Y = 0, Z = 0 } }.ToList();
                leftMerges.Clear();

                foreach (var abstractMerge in abstractMerges)
                {
                    if (abstractMerge.Z < bombCluster.clusterWidth / 2)
                        leftMerges.Add(new { listPos = abstractMerge.listPos, X = abstractMerge.X, Y = abstractMerge.Y, Z = abstractMerge.Z });
                    else
                        rightMerges.Add(new { listPos = abstractMerge.listPos, X = abstractMerge.X, Y = abstractMerge.Y, Z = abstractMerge.Z });
                }

                leftMerges.Sort((a, b) =>
                {
                    int result = b.Y.CompareTo(a.Y);
                    if (result == 0)
                    {
                        result = a.Z.CompareTo(b.Z);
                        if (result == 0)
                        {
                            result = a.X.CompareTo(b.X);
                        }
                    }
                    return result;
                });

                rightMerges.Sort((a, b) =>
                {
                    int result = b.Y.CompareTo(a.Y);
                    if (result == 0)
                    {
                        result = b.Z.CompareTo(a.Z);
                        if (result == 0)
                        {
                            result = b.X.CompareTo(a.X);
                        }
                    }
                    return result;
                });

                bombCluster.bombs = new List<Bomb>();
                for (int i = 0; i < Math.Max(leftMerges.Count, rightMerges.Count); i++)
                {
                    if (i < leftMerges.Count)
                    {
                        IMyShipMergeBlock actualMerge = merges[leftMerges[i].listPos];
                        IMyWarhead warheadOnTop = CubeGrid.GetCubeBlock(new Vector3I(actualMerge.Position.X + warheadMergeOffset[0], actualMerge.Position.Y + warheadMergeOffset[1], actualMerge.Position.Z + warheadMergeOffset[2])).FatBlock as IMyWarhead;
                        bombCluster.bombs.Add(new Bomb(actualMerge, warheadOnTop));
                    }
                    if (i < rightMerges.Count)
                    {
                        IMyShipMergeBlock actualMerge = merges[rightMerges[i].listPos];
                        IMyWarhead warheadOnTop = CubeGrid.GetCubeBlock(new Vector3I(actualMerge.Position.X + warheadMergeOffset[0], actualMerge.Position.Y + warheadMergeOffset[1], actualMerge.Position.Z + warheadMergeOffset[2])).FatBlock as IMyWarhead;
                        bombCluster.bombs.Add(new Bomb(actualMerge, warheadOnTop));
                    }
                }
            }

            //CURSED SPLIT SEQUENCE SORT ALGORITHM END
            //--------------------------------------------------------------------------------------------------------------------

            public class FlightComputer
            {
                public FlightComputer(Missile parent)
                {
                    this.parent = parent;
                }

                Missile parent;

                public float missileMass, maxThrust;
                public Vector3 lastKnownGravity = new Vector3(0, 0, 0);

                public float evasiveManeuversStrengh = 0;
                public float currentEvasiveManeuversRotation = 0;

                public float sideMomentumCorrectionFactor = 0.1F;
                public float downMomentumCorrectionFactor = 0.1F;
                public float upMomentumCorrectionFactor = 0.1F;

                public void UpdateFlightInfo()
                {
                    IMyRemoteControl remote = parent.TryGetWorkingRemote();
                    if (remote != null)
                    {
                        lastKnownGravity = remote.GetNaturalGravity();
                        missileMass = remote.CalculateShipMass().BaseMass;
                    }

                    maxThrust = 0;
                    foreach (IMyThrust thruster in parent.thrusters)
                        if (thruster != null && thruster.IsWorking)
                            maxThrust += 98400;
                }

                public void FlyTo(Vector3 enemyPos)
                {
                    Vector3 enemyVector = enemyPos - parent.CubeGrid.GetPosition();
                    SetGyroscopesToVector(CalculateThrustDirection(enemyVector));
                }

                //--------------------------------------------------------------------------------------------------------------------
                //FLIGHT VECTOR CALCULATION START

                public Vector3 CalculateThrustDirection(Vector3 desiredDirection)
                {
                    float availableThrust = maxThrust;
                    Vector3 gravity = lastKnownGravity;

                    Vector3 entireCorrection = CalculateShipCorrection(ref availableThrust, desiredDirection, gravity);
                    if (desiredDirection.Length() == 0 || availableThrust == 0)
                        return entireCorrection;

                    Vector3 modifiedDesiredDirection = Vector3.Normalize(desiredDirection) * availableThrust / missileMass;

                    return modifiedDesiredDirection + entireCorrection;
                }

                Vector3 CalculateShipCorrection(ref float availableThrust, Vector3 desiredDirection, Vector3 gravity)
                {
                    Vector3 gravityCorrection = CalculateGravityCorrection(desiredDirection, gravity);
                    Vector3 momentumCorrection = CalculateMomentumCorrection(desiredDirection);

                    return CalculateEntireCorrection(ref availableThrust, gravityCorrection, momentumCorrection);
                }

                Vector3 CalculateGravityCorrection(Vector3 desiredDirection, Vector3 gravity)
                {
                    if (desiredDirection.Length() == 0 || gravity.Length() == 0)
                        return -gravity;

                    return -Vector3.ProjectOnPlane(ref gravity, ref desiredDirection);
                }

                Vector3 CalculateMomentumCorrection(Vector3 desiredDirection)
                {
                    Vector3 momentum = parent.CubeGrid.LinearVelocity;

                    if (desiredDirection.Length() == 0)
                        return -parent.CubeGrid.LinearVelocity;

                    return -Vector3.ProjectOnPlane(ref momentum, ref desiredDirection);
                }

                Vector3 CalculateEntireCorrection(ref float availableThrust, Vector3 gravityCorrection, Vector3 momentumCorrection)
                {
                    if (gravityCorrection.Length() == 0 || momentumCorrection.Length() == 0)
                        return gravityCorrection + momentumCorrection * sideMomentumCorrectionFactor;

                    Vector3 gravityMomentumCorrection = Vector3.ProjectOnVector(ref momentumCorrection, ref gravityCorrection);
                    Vector3 sideMomentumCorrection = (momentumCorrection - gravityMomentumCorrection) * sideMomentumCorrectionFactor;

                    if (Vector3.Dot(gravityMomentumCorrection, gravityCorrection) < 0)
                        gravityMomentumCorrection *= upMomentumCorrectionFactor;
                    else
                        gravityMomentumCorrection *= downMomentumCorrectionFactor;

                    Vector3 entireGravityDirCorrection = gravityMomentumCorrection + gravityCorrection;

                    availableThrust = Math.Max(0, availableThrust - (float)entireGravityDirCorrection.Length() * missileMass);

                    if ((float)sideMomentumCorrection.Length() * missileMass > availableThrust)
                        momentumCorrection *= availableThrust / ((float)sideMomentumCorrection.Length() * missileMass);

                    availableThrust = availableThrust - (float)sideMomentumCorrection.Length() * missileMass;

                    return entireGravityDirCorrection + sideMomentumCorrection;
                }

                //FLIGHT VECTOR CALCULATION END
                //--------------------------------------------------------------------------------------------------------------------

                public void SetGyroscopesToVector(Vector3 vector)
                {
                    Vector3 targetVector = Vector3.Normalize(vector);
                    foreach (IMyGyro gyro in parent.gyros)
                    {
                        gyro.Yaw = 2 * (float)Math.Asin(Vector3.Dot(targetVector, gyro.WorldMatrix.Right) / targetVector.Length());
                        gyro.Pitch = 2 * (float)Math.Asin(Vector3.Dot(targetVector, gyro.WorldMatrix.Down) / targetVector.Length());
                    }
                }
            }

            public class TargetingSystem
            {
                public TargetingSystem(Missile parent)
                {
                    missileGuidance = new MissileGuidance(this);
                    bombingComputer = new BombingComputer(this);

                    this.parent = parent;
                }

                public MissileGuidance missileGuidance;
                public BombingComputer bombingComputer;

                Missile parent;

                public MyDetectedEntityInfo detectedEnemy;

                public IMyTurretControlBlock TryGetWorkingTargetControl()
                {
                    return parent.turretControls.FirstOrDefault(controller => controller != null && controller.IsWorking);
                }

                public void UpdateEnemyInfo()
                {
                    IMyTurretControlBlock turretControl = TryGetWorkingTargetControl();
                    if (turretControl != null)
                        detectedEnemy = turretControl.GetTargetedEntity();
                }

                public class MissileGuidance
                {
                    public MissileGuidance(TargetingSystem parent)
                    {
                        this.parent = parent;
                    }

                    TargetingSystem parent;

                    public Vector3 ProportionalNavigation(MyDetectedEntityInfo enemy) //https://www.youtube.com/watch?v=MpUUsDDE1sI&list=LL&index=1 excellent proNav material
                    {
                        Vector3 missileVelocity = parent.parent.CubeGrid.LinearVelocity;
                        Vector3 enemyPosition = enemy.Position - parent.parent.CubeGrid.GetPosition();
                        Vector3 enemyVelocity = enemy.Velocity - missileVelocity;

                        float hitTime = SolveQuadraticEquation(Vector3.Dot(enemyVelocity, enemyVelocity) - missileVelocity.LengthSquared(), 2 * Vector3.Dot(enemyPosition, enemyVelocity), Vector3.Dot(enemyPosition, enemyPosition));
                        if (hitTime == 0)
                            return new Vector3(0, 0, 0);

                        return (enemyPosition + (enemyVelocity + missileVelocity) * hitTime);
                    }

                    public float SolveQuadraticEquation(float a, float b, float c)
                    {
                        float delta = b * b - 4 * a * c;
                        if (delta < 0)
                            return 0;
                        else if (delta == 0)
                            return -b / (2 * a);
                        else
                            return (-b - (float)Math.Sqrt(delta)) / (2 * a);
                    }
                }

                public class BombingComputer
                {
                    public BombingComputer(TargetingSystem parent)
                    {
                        this.parent = parent;
                    }

                    TargetingSystem parent;

                    private float maxSpeed = 100;

                    public Vector3[] GetSideMostClusterHits(List<IMyShipMergeBlock> clusterMerges, Vector3 enemyPos)
                    {
                        BombCluster bombCluster = parent.parent.bombCluster;
                        IMyShipController controller = parent.parent.TryGetWorkingRemote();
                        Vector3 terminationPlaneNormal = enemyPos - parent.parent.CubeGrid.GetPosition();
                        Vector3 gravity = controller.GetNaturalGravity();

                        Vector3 hitPosOne = GetEntityEndPosition(bombCluster.bombs[0].warhead, controller.GetShipVelocities().AngularVelocity, gravity, terminationPlaneNormal, gravity);
                        Vector3 hitPosTwo = GetEntityEndPosition(bombCluster.bombs[1].warhead, controller.GetShipVelocities().AngularVelocity, gravity, terminationPlaneNormal, gravity);
                        Vector3 hitPosThree = GetEntityEndPosition(bombCluster.bombs[bombCluster.clusterWidth - 1].warhead, controller.GetShipVelocities().AngularVelocity, gravity, terminationPlaneNormal, gravity);
                        Vector3 hitPosFour = GetEntityEndPosition(bombCluster.bombs[bombCluster.clusterWidth - 2].warhead, controller.GetShipVelocities().AngularVelocity, gravity, terminationPlaneNormal, gravity);

                        return new Vector3[] { hitPosOne, hitPosTwo };
                    }

                    //--------------------------------------------------------------------------------------------------------------------
                    //TRAJECTORY PREDICTION (ITERATIVE SIMULATION + RUNGE-KUTTA + CURSED MODIFIERS) START

                    public Vector3 GetEntityEndPosition(IMyTerminalBlock entity, Vector3 angularVelocity, Vector3 acceleration, Vector3 gravity, Vector3 target, float maxSimTime = 60)
                    {
                        Vector3 velocity = Vector3.ClampToSphere(entity.CubeGrid.LinearVelocity + AngularToLinearVelocity(entity, angularVelocity), maxSpeed * 5 / 4);
                        Vector3 position = entity.GetPosition() + velocity * 0.5F;

                        Vector3 targetVector = target - position;
                        Vector3 gravityVector = gravity.Length() == 0 ? gravity : Vector3.ProjectOnVector(ref targetVector, ref gravity);

                        for (float i = 0; i < maxSimTime && i < 60; i += 1)
                        {
                            Vector3 k1v = velocity + acceleration;
                            Vector3 k2v = velocity + k1v / 2 + acceleration * 1.5F;
                            Vector3 k3v = velocity + k2v / 2 + acceleration * 1.5F;
                            Vector3 k4v = velocity + k3v + acceleration * 2;

                            velocity = Vector3.ClampToSphere((k1v + 2 * k2v + 2 * k3v + k4v) / 6, maxSpeed * 5 / 4);
                            position += velocity;

                            Vector3 newTargetVector = target - position;
                            if (Vector3.Dot(targetVector, newTargetVector) < 0)
                                break;

                            Vector3 newGraviyVector = gravity.Length() == 0 ? gravity : Vector3.ProjectOnVector(ref newTargetVector, ref gravity);
                            if (Vector3.Dot(gravityVector, newGraviyVector) < 0)
                                break;
                        }

                        return position;
                    }

                    //TRAJECTORY PREDICTION (ITERATIVE SIMULATION + RUNGE-KUTTA + CURSED MODIFIERS) END
                    //--------------------------------------------------------------------------------------------------------------------

                    public Vector3 AngularToLinearVelocity(IMyTerminalBlock entity, Vector3 angularVelocity)
                    {
                        IMyCubeGrid CubeGrid = entity.CubeGrid;
                        Vector3 volumeVector = CubeGrid.Max - CubeGrid.Min;

                        Vector3 entityAbstractPosition, gridCenterAtEntityHeight;
                        if (volumeVector.X > volumeVector.Y && volumeVector.X > volumeVector.Z)
                        {
                            entityAbstractPosition = new Vector3(entity.Position.Y, entity.Position.X, entity.Position.Z);
                            gridCenterAtEntityHeight = new Vector3(CubeGrid.Min.Y + volumeVector.Y / 2, entity.Position.X, CubeGrid.Min.Z + volumeVector.Z / 2);
                        }
                        else if (volumeVector.Z > volumeVector.Y && volumeVector.Z > volumeVector.X)
                        {
                            entityAbstractPosition = new Vector3(entity.Position.X, entity.Position.Z, entity.Position.Y);
                            gridCenterAtEntityHeight = new Vector3(CubeGrid.Min.X + volumeVector.X / 2, entity.Position.Z, CubeGrid.Min.Y + volumeVector.Y / 2);
                        }
                        else
                        {
                            entityAbstractPosition = new Vector3(entity.Position.X, entity.Position.Y, entity.Position.Z);
                            gridCenterAtEntityHeight = new Vector3(CubeGrid.Min.X + volumeVector.X / 2, entity.Position.Y, CubeGrid.Min.Z + volumeVector.Z / 2);
                        }

                        Vector3 radius = Vector3.TransformNormal(entityAbstractPosition - gridCenterAtEntityHeight, entity.WorldMatrix);
                        return Vector3.ClampToSphere(Vector3.Cross(angularVelocity, radius), 1200 / CubeGrid.LinearVelocity.Length());
                    }
                }
            }
        }

        public class Launcher
        {

        }

        //!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!
        //DEBUG API SECTION START (required SE debug api mod)
        //!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!

        public class PIDController
        {
            private double Kp;
            private double Ki;
            private double Kd; 
            private double integral;
            private double previousError;

            public PIDController(double kp, double ki, double kd)
            {
                Kp = kp;
                Ki = ki;
                Kd = kd;
                integral = 0;
                previousError = 0;
            }

            public double Calculate(double error, double dt)
            {
                double proportional = Kp * error;

                integral += error * dt;
                double integralTerm = Ki * integral;

                double derivative = Kd * (error - previousError) / dt;
                previousError = error;

                return proportional + integralTerm + derivative;
            }

            public void Reset()
            {
                integral = 0;
                previousError = 0;
            }
        }


        public class DebugAPI
        {
            public readonly bool ModDetected;

            /// <summary>
            /// Changing this will affect OnTop draw for all future draws that don't have it specified.
            /// </summary>
            public bool DefaultOnTop;

            /// <summary>
            /// Recommended to be used at start of Main(), unless you wish to draw things persistently and remove them manually.
            /// <para>Removes everything except AdjustNumber and chat messages.</para>
            /// </summary>
            public void RemoveDraw() => _removeDraw?.Invoke(_pb);
            Action<IMyProgrammableBlock> _removeDraw;

            /// <summary>
            /// Removes everything that was added by this API (except chat messages), including DeclareAdjustNumber()!
            /// <para>For calling in Main() you should use <see cref="RemoveDraw"/> instead.</para>
            /// </summary>
            public void RemoveAll() => _removeAll?.Invoke(_pb);
            Action<IMyProgrammableBlock> _removeAll;

            /// <summary>
            /// You can store the integer returned by other methods then remove it with this when you wish.
            /// <para>Or you can not use this at all and call <see cref="RemoveDraw"/> on every Main() so that your drawn things live a single PB run.</para>
            /// </summary>
            public void Remove(int id) => _remove?.Invoke(_pb, id);
            Action<IMyProgrammableBlock, int> _remove;

            public int DrawPoint(Vector3D origin, Color color, float radius = 0.2f, float seconds = DefaultSeconds, bool? onTop = null) => _point?.Invoke(_pb, origin, color, radius, seconds, onTop ?? DefaultOnTop) ?? -1;
            Func<IMyProgrammableBlock, Vector3D, Color, float, float, bool, int> _point;

            public int DrawLine(Vector3D start, Vector3D end, Color color, float thickness = DefaultThickness, float seconds = DefaultSeconds, bool? onTop = null) => _line?.Invoke(_pb, start, end, color, thickness, seconds, onTop ?? DefaultOnTop) ?? -1;
            Func<IMyProgrammableBlock, Vector3D, Vector3D, Color, float, float, bool, int> _line;

            public int DrawAABB(BoundingBoxD bb, Color color, Style style = Style.Wireframe, float thickness = DefaultThickness, float seconds = DefaultSeconds, bool? onTop = null) => _aabb?.Invoke(_pb, bb, color, (int)style, thickness, seconds, onTop ?? DefaultOnTop) ?? -1;
            Func<IMyProgrammableBlock, BoundingBoxD, Color, int, float, float, bool, int> _aabb;

            public int DrawOBB(MyOrientedBoundingBoxD obb, Color color, Style style = Style.Wireframe, float thickness = DefaultThickness, float seconds = DefaultSeconds, bool? onTop = null) => _obb?.Invoke(_pb, obb, color, (int)style, thickness, seconds, onTop ?? DefaultOnTop) ?? -1;
            Func<IMyProgrammableBlock, MyOrientedBoundingBoxD, Color, int, float, float, bool, int> _obb;

            public int DrawSphere(BoundingSphereD sphere, Color color, Style style = Style.Wireframe, float thickness = DefaultThickness, int lineEveryDegrees = 15, float seconds = DefaultSeconds, bool? onTop = null) => _sphere?.Invoke(_pb, sphere, color, (int)style, thickness, lineEveryDegrees, seconds, onTop ?? DefaultOnTop) ?? -1;
            Func<IMyProgrammableBlock, BoundingSphereD, Color, int, float, int, float, bool, int> _sphere;

            public int DrawMatrix(MatrixD matrix, float length = 1f, float thickness = DefaultThickness, float seconds = DefaultSeconds, bool? onTop = null) => _matrix?.Invoke(_pb, matrix, length, thickness, seconds, onTop ?? DefaultOnTop) ?? -1;
            Func<IMyProgrammableBlock, MatrixD, float, float, float, bool, int> _matrix;

            /// <summary>
            /// Adds a HUD marker for a world position.
            /// <para>White is used if <paramref name="color"/> is null.</para>
            /// </summary>
            public int DrawGPS(string name, Vector3D origin, Color? color = null, float seconds = DefaultSeconds) => _gps?.Invoke(_pb, name, origin, color, seconds) ?? -1;
            Func<IMyProgrammableBlock, string, Vector3D, Color?, float, int> _gps;

            /// <summary>
            /// Adds a notification center on screen. Do not give 0 or lower <paramref name="seconds"/>.
            /// </summary>
            public int PrintHUD(string message, Font font = Font.Debug, float seconds = 2) => _printHUD?.Invoke(_pb, message, font.ToString(), seconds) ?? -1;
            Func<IMyProgrammableBlock, string, string, float, int> _printHUD;

            /// <summary>
            /// Shows a message in chat as if sent by the PB (or whoever you want the sender to be)
            /// <para>If <paramref name="sender"/> is null, the PB's CustomName is used.</para>
            /// <para>The <paramref name="font"/> affects the fontface and color of the entire message, while <paramref name="senderColor"/> only affects the sender name's color.</para>
            /// </summary>
            public void PrintChat(string message, string sender = null, Color? senderColor = null, Font font = Font.Debug) => _chat?.Invoke(_pb, message, sender, senderColor, font.ToString());
            Action<IMyProgrammableBlock, string, string, Color?, string> _chat;

            /// <summary>
            /// Used for realtime adjustments, allows you to hold the specified key/button with mouse scroll in order to adjust the <paramref name="initial"/> number by <paramref name="step"/> amount.
            /// <para>Add this once at start then store the returned id, then use that id with <see cref="GetAdjustNumber(int)"/>.</para>
            /// </summary>
            public void DeclareAdjustNumber(out int id, double initial, double step = 0.05, Input modifier = Input.Control, string label = null) => id = _adjustNumber?.Invoke(_pb, initial, step, modifier.ToString(), label) ?? -1;
            Func<IMyProgrammableBlock, double, double, string, string, int> _adjustNumber;

            /// <summary>
            /// See description for: <see cref="DeclareAdjustNumber(double, double, Input, string)"/>.
            /// <para>The <paramref name="noModDefault"/> is returned when the mod is not present.</para>
            /// </summary>
            public double GetAdjustNumber(int id, double noModDefault = 1) => _getAdjustNumber?.Invoke(_pb, id) ?? noModDefault;
            Func<IMyProgrammableBlock, int, double> _getAdjustNumber;

            /// <summary>
            /// Gets simulation tick since this session started. Returns -1 if mod is not present.
            /// </summary>
            public int GetTick() => _tick?.Invoke() ?? -1;
            Func<int> _tick;

            /// <summary>
            /// Gets time from Stopwatch which is accurate to nanoseconds, can be used to measure code execution time.
            /// Returns TimeSpan.Zero if mod is not present.
            /// </summary>
            public TimeSpan GetTimestamp() => _timestamp?.Invoke() ?? TimeSpan.Zero;
            Func<TimeSpan> _timestamp;

            /// <summary>
            /// Use with a using() statement to measure a chunk of code and get the time difference in a callback.
            /// <code>
            /// using(Debug.Measure((t) => Echo($"diff={t}")))
            /// {
            ///    // code to measure
            /// }
            /// </code>
            /// This simply calls <see cref="GetTimestamp"/> before and after the inside code.
            /// </summary>
            public MeasureToken Measure(Action<TimeSpan> call) => new MeasureToken(this, call);

            /// <summary>
            /// <see cref="Measure(Action{TimeSpan})"/>
            /// </summary>
            public MeasureToken Measure(string prefix) => new MeasureToken(this, (t) => PrintHUD($"{prefix} {t.TotalMilliseconds} ms"));

            public struct MeasureToken : IDisposable
            {
                DebugAPI API;
                TimeSpan Start;
                Action<TimeSpan> Callback;

                public MeasureToken(DebugAPI api, Action<TimeSpan> call)
                {
                    API = api;
                    Callback = call;
                    Start = API.GetTimestamp();
                }

                public void Dispose()
                {
                    Callback?.Invoke(API.GetTimestamp() - Start);
                }
            }

            public enum Style { Solid, Wireframe, SolidAndWireframe }
            public enum Input { MouseLeftButton, MouseRightButton, MouseMiddleButton, MouseExtraButton1, MouseExtraButton2, LeftShift, RightShift, LeftControl, RightControl, LeftAlt, RightAlt, Tab, Shift, Control, Alt, Space, PageUp, PageDown, End, Home, Insert, Delete, Left, Up, Right, Down, D0, D1, D2, D3, D4, D5, D6, D7, D8, D9, A, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T, U, V, W, X, Y, Z, NumPad0, NumPad1, NumPad2, NumPad3, NumPad4, NumPad5, NumPad6, NumPad7, NumPad8, NumPad9, Multiply, Add, Separator, Subtract, Decimal, Divide, F1, F2, F3, F4, F5, F6, F7, F8, F9, F10, F11, F12 }
            public enum Font { Debug, White, Red, Green, Blue, DarkBlue }

            const float DefaultThickness = 0.02f;
            const float DefaultSeconds = -1;

            IMyProgrammableBlock _pb;

            /// <summary>
            /// NOTE: if mod is not present then methods will simply not do anything, therefore you can leave the methods in your released code.
            /// </summary>
            /// <param name="program">pass `this`.</param>
            /// <param name="drawOnTopDefault">set the default for onTop on all objects that have such an option.</param>
            public DebugAPI(MyGridProgram program, bool drawOnTopDefault = false)
            {
                if (program == null) throw new Exception("Pass `this` into the API, not null.");

                DefaultOnTop = drawOnTopDefault;
                _pb = program.Me;

                var methods = _pb.GetProperty("DebugAPI")?.As<IReadOnlyDictionary<string, Delegate>>()?.GetValue(_pb);
                if (methods != null)
                {
                    Assign(out _removeAll, methods["RemoveAll"]);
                    Assign(out _removeDraw, methods["RemoveDraw"]);
                    Assign(out _remove, methods["Remove"]);
                    Assign(out _point, methods["Point"]);
                    Assign(out _line, methods["Line"]);
                    Assign(out _aabb, methods["AABB"]);
                    Assign(out _obb, methods["OBB"]);
                    Assign(out _sphere, methods["Sphere"]);
                    Assign(out _matrix, methods["Matrix"]);
                    Assign(out _gps, methods["GPS"]);
                    Assign(out _printHUD, methods["HUDNotification"]);
                    Assign(out _chat, methods["Chat"]);
                    Assign(out _adjustNumber, methods["DeclareAdjustNumber"]);
                    Assign(out _getAdjustNumber, methods["GetAdjustNumber"]);
                    Assign(out _tick, methods["Tick"]);
                    Assign(out _timestamp, methods["Timestamp"]);

                    RemoveAll(); // cleanup from past compilations on this same PB

                    ModDetected = true;
                }
            }

            void Assign<T>(out T field, object method) => field = (T)method;
        }

        //!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!
        //DEBUG API SECTION END
        //!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!
    }
}
