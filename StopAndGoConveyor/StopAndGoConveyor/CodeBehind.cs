using ABB.Robotics.Math;
using ABB.Robotics.RobotStudio;
using ABB.Robotics.RobotStudio.Stations;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;
using System.Text;

namespace StopAndGoConveyor
{
    /// <summary>
    /// Code-behind class for the StopAndGoConveyor Smart Component.
    /// </summary>
    /// <remarks>
    /// The code-behind class should be seen as a service provider used by the 
    /// Smart Component runtime. Only one instance of the code-behind class
    /// is created, regardless of how many instances there are of the associated
    /// Smart Component.
    /// Therefore, the code-behind class should not store any state information.
    /// Instead, use the SmartComponent.StateCache collection.
    /// </remarks>
    public class CodeBehind : SmartComponentCodeBehind
    {
        const string StopperGroup = "Stoppers";

        //public override void OnLoad(SmartComponent component)
        //{
        //    base.OnLoad(component);
        //    component.DisconnectFromLibrary();
        //    Logger.AddMessage("loaded");
        //}

        /// <summary>
        /// Called when the value of a dynamic property value has changed.
        /// </summary>
        /// <param name="component"> Component that owns the changed property. </param>
        /// <param name="changedProperty"> Changed property. </param>
        /// <param name="oldValue"> Previous value of the changed property. </param>
        public override void OnPropertyValueChanged(SmartComponent component, DynamicProperty changedProperty, Object oldValue)
        {
            HandlePropertyEvent(component, changedProperty, oldValue);
        }

        /// <summary>
        /// Called when the value of an I/O signal value has changed.
        /// </summary>
        /// <param name="component"> Component that owns the changed signal. </param>
        /// <param name="changedSignal"> Changed signal. </param>
        public override void OnIOSignalValueChanged(SmartComponent component, IOSignal changedSignal)
        {
            if (IsHigh(changedSignal))
            {
                HandleSignalEvent(changedSignal, component);
            }
        }

        /// <summary>
        /// Called during simulation.
        /// </summary>
        /// <param name="component"> Simulated component. </param>
        /// <param name="simulationTime"> Time (in ms) for the current simulation step. </param>
        /// <param name="previousTime"> Time (in ms) for the previous simulation step. </param>
        /// <remarks>
        /// For this method to be called, the component must be marked with
        /// simulate="true" in the xml file.
        /// </remarks>
        public override void OnSimulationStep(SmartComponent component, double simulationTime, double previousTime)
        {
            var enable = component.IOSignals["Enable"];
            if (IsHigh(enable))
            {
                HandleSimulationStep(component, simulationTime, previousTime);
            }
        }


        //Simulation handling
        void HandleSimulationStep(SmartComponent component, double simulationTime, double previousTime)
        {
            double speed = (double)component.Properties["Speed"].Value;
            double lenght = (double)component.Properties["Lenght"].Value;
            var objectlist = component.GraphicComponents["ObjectList"] as GraphicComponentGroup;
            int nbObjects = objectlist.GraphicComponents.Count;
            if (speed == 0) return;
            if (lenght == 0) return;
            if (nbObjects == 0) return;
            double deltaTime = (simulationTime - previousTime);
            double distanceToMove = speed * (deltaTime / 1000.0);
            var toDelete = new bool[nbObjects];
            var partColliding = new bool[nbObjects];
            int nbStoppers = (int)component.Properties["Stopper"].Value;
            var partAtStopper = new int[nbObjects];

            for (int i = 0; i < nbObjects; i++)
            {
                //check if part is at stopper
                var part = objectlist.GraphicComponents[i] as Part;
                for (int j = 0; j < nbStoppers; j++)
                {
                    var stopperCommand = component.IOSignals["Stopper" + (j + 1).ToString() + "Command"];
                    var stopperPosition = (double)component.Properties["Stopper" + (j + 1).ToString() + "Position"].Value;
                    if (IsHigh(stopperCommand) && IsCollidingDistance(component, part, stopperPosition))
                    {
                        partAtStopper[i] = j + 1;
                    }
                }
                //check if part colliding with previous part
                if (i > 0)
                {
                    var previousPart = objectlist.GraphicComponents[i - 1] as Part;
                    if (IsCollidingPart(previousPart, part))
                    {
                        partColliding[i] = true;
                    }
                }
                //check if part is further than lenght
                if (GetDistance(component, part) > lenght)
                {
                    toDelete[i] = true;
                }
                else
                {
                    toDelete[i] = false;
                }

            }

            int nPart= 0;
            //apply movements // deletions // update stopper sensors
            for (int i = 0; i < nbObjects; i++)
            {
                var part = objectlist.GraphicComponents[nPart] as Part;
                if (!partColliding[i] && (partAtStopper[i] == 0))
                {
                    ApplyOffset(component, part, distanceToMove);
                }

                if (toDelete[i])
                {
                    objectlist.GraphicComponents.Remove(part, true);
                    part.Delete();
                }
                else
                {
                    nPart++;
                }

            }

            //update stopper sensors
            var stoppersWithPart= partAtStopper.Distinct().ToList();
            for (int i = 0; i < nbStoppers; i++)
            {
                var stopperSensor = component.IOSignals["Stopper" + (i + 1).ToString() + "Sensor"];
                if (stoppersWithPart.Contains(i + 1))
                {
                    stopperSensor.Value = 1;
                }
                else
                {
                    stopperSensor.Value = 0;
                }
            }


        }

        //Property handling
        void HandlePropertyEvent(SmartComponent component, DynamicProperty changedProperty, Object oldValue)
        {
            if (changedProperty.Name == "Stopper")
            {
                component.DisconnectFromLibrary();
                int numberOfStoppers = (int)changedProperty.Value;
                //Remove existing stoppers
                int oldNumberOfStoppers = (int)oldValue;
                if (oldNumberOfStoppers > 0)
                {

                    for (int i = 1; i <= oldNumberOfStoppers; i++)
                    {
                        component.IOSignals.Remove("Stopper" + i + "Command");
                        component.IOSignals.Remove("Stopper" + i + "Sensor");
                        component.Properties.Remove("Stopper" + i + "Position");
                    }

                }
                if (numberOfStoppers < 1) return;

                //add new stoppers
                for (int i = 1; i <= numberOfStoppers; i++)
                {
                    var signalStopperCommand = new IOSignal("Stopper" + i + "Command", IOSignalType.DigitalInput);
                    var signalStopperSensor = new IOSignal("Stopper" + i + "Sensor", IOSignalType.DigitalOutput);
                    signalStopperCommand.GroupName = StopperGroup;
                    signalStopperSensor.GroupName = StopperGroup;
                    component.IOSignals.Add(signalStopperCommand);
                    component.IOSignals.Add(signalStopperSensor);

                    var propStopperPosition = new DynamicProperty("Stopper" + i + "Position", "System.Double");
                    propStopperPosition.GroupName = StopperGroup;
                    propStopperPosition.Attributes.Add("AutoApply", "True");
                    propStopperPosition.Attributes.Add("Quantity", "Length");
                    component.Properties.Add(propStopperPosition);

                }
            }
        }

        //Signal handling
        void HandleSignalEvent(IOSignal signal, SmartComponent component)
        {
            if (signal.Name == "AddObject")
            {
                var group = component.GraphicComponents["ObjectList"] as GraphicComponentGroup;
                var part = component.Properties["SourcePart"].Value as Part;
                if (part is null) return;
                var newPart = part.Copy() as Part;
                newPart.Transform.GlobalMatrix = Matrix4.Identity;
                group.GraphicComponents.Add(newPart);
                newPart.Visible = true;
            }
            else if (signal.Name == "Clear")
            {
                var group = component.GraphicComponents["ObjectList"] as GraphicComponentGroup;
                group.GraphicComponents.Clear(true);
            }
        }


        //Helpers functions
        bool IsHigh(IOSignal signal)
        {
            return (int)signal.Value > 0;
        }

        void ApplyOffset(SmartComponent reference, Part part, double offset)
        {
            var distance = GetDistance(reference, part);
            distance += offset;
            var newPose = reference.Transform.GlobalMatrix;
            newPose.TranslateLocal(distance, 0, 0);
            part.Transform.GlobalMatrix = newPose;
        }

        double GetDistance(SmartComponent reference, Part part)
        {
            return reference.Transform.GlobalMatrix.Translation.Distance(part.Transform.GlobalMatrix.Translation);
        }

        bool IsCollidingPart(Part part1, Part part2)
        {
            return CollisionDetector.CheckCollision(part1, part2, 0) == CollisionType.Collision;
        }

        bool IsCollidingDistance(SmartComponent reference, Part part, double distance)
        {
            double tolerance = 0.001;
            var box = new BoundingBox(
                new Vector3(distance, -100000, -100000),
                new Vector3(distance + tolerance, +100000, +10000)
                );
            var intersect = part.IntersectVolume(box, reference.Transform.GlobalMatrix);
            return (intersect != IntersectionType.Outside);
        }

    }
}

