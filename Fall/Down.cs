using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace Fall
{
    public class Down
    {
        static double RadDeg = Math.PI / 180.0;
        public static double distance;
        public static double Bt;
        static double Q, eas, pLift, pDrag, lift, liftUp, drag = 0;
        bool showWindow = true;
        static Rect windowPos = new Rect(100, 100, 0, 0);

        static Vessel Ship = FlightGlobals.ActiveVessel;
        static CelestialBody body = Ship.mainBody;
        static SurfaceObject LS = new SurfaceObject();

        static double gravity(double Gp, double R)
        {
            return Gp / (R * R);
        }

        static double density(double P, double T, double R)
        {
            return P / (T * R);
        }

        static void GetAeroStats(Vessel v, out double thrust, out double maxThrust, out double maxFuelFlow, out double CdS)  //adapted from AeroGUI by Nathankell - 2015
        {
            Vector3d nVel = v.srf_velocity.normalized;
            thrust = maxThrust = maxFuelFlow = 0;
            Vector3d vLift = Vector3d.zero; // the sum of lift from all parts
            Vector3d vDrag = Vector3d.zero; // the sum of drag from all parts
            double sqrMag = v.srf_velocity.sqrMagnitude;
            Q = 0.5 * v.atmDensity * sqrMag; // dynamic pressure, aka Q
            eas = Math.Sqrt(sqrMag * v.atmDensity / 1.225); // Equivalent Air Speed
            // i.e. your airspeed at sea level with the same Q
            double dTime = TimeWarp.fixedDeltaTime;

            // Now we loop through all parts, checking the modules in each part
            // This way we get all drag, lift, and thrust.
            for (int i = 0; i < v.Parts.Count; ++i)
            {
                Part p = v.Parts[i];

                // get part drag (but not wing/surface drag)
                vDrag += -p.dragVectorDir * p.dragScalar;
                if (!p.hasLiftModule)
                {
                    Vector3 bodyLift = p.transform.rotation * (p.bodyLiftScalar * p.DragCubes.LiftForce);
                    bodyLift = Vector3.ProjectOnPlane(bodyLift, -p.dragVectorDir);
                    vLift += bodyLift;
                }

                // now find modules
                for (int j = 0; j < p.Modules.Count; ++j)
                {
                    var m = p.Modules[j];
                    if (m is ModuleLiftingSurface) // control surface derives from this
                    {
                        ModuleLiftingSurface wing = (ModuleLiftingSurface)m;
                        vLift += wing.liftForce;
                        vDrag += wing.dragForce;
                    }
                    // Get thrust, maxThrust, fuelFlow
                    if (m is ModuleEngines) // FX derives from this
                    {
                        thrust += ((ModuleEngines)m).finalThrust;
                        maxThrust += ((ModuleEngines)m).maxThrust;
                        maxFuelFlow += ((ModuleEngines)m).maxFuelFlow;
                    }
                }
            }
            // pLift is 'pure' lift, and same for drag, i.e. just the magnitude of each
            pLift = vLift.magnitude;
            pDrag = vDrag.magnitude;
            Vector3d force = vLift + vDrag; // sum of all forces on the craft
            Vector3d liftDir = -Vector3d.Cross(v.transform.right, nVel); // we need the "lift" direction, which
            // is "up" from our current velocity vector and roll angle.

            // Now we can compute the dots.
            lift = Vector3d.Dot(force, liftDir); // just the force in the 'lift' direction
            liftUp = Vector3d.Dot(force, v.upAxis); // just the force in the 'up' direction (note, some of it may be drag!)
            drag = Vector3d.Dot(force, -nVel); // drag force, = pDrag + lift-induced drag
            CdS = drag * 1000.0 / Q;
        }

        static double atmTempOffset() // this code from NathanKell - 2015
        {
            Vector3 sunVector = (FlightGlobals.Bodies[0].scaledBody.transform.position - ScaledSpace.LocalToScaledSpace(Ship.transform.position)).normalized;
            double sunDot = Vector3.Dot(sunVector, Ship.upAxis);
            Vector3 mainBodyUp = Ship.mainBody.bodyTransform.up;
            float sunAxialDot = Vector3.Dot(sunVector, mainBodyUp);
            double bodyPolarAngle = Math.Acos(Vector3.Dot(mainBodyUp, Ship.upAxis));
            double sunPolarAngle = Math.Acos(sunAxialDot);
            double sunBodyMaxDot = (1d + Math.Cos(sunPolarAngle - bodyPolarAngle)) * 0.5d;
            double sunBodyMinDot = (1d + Math.Cos(sunPolarAngle + bodyPolarAngle)) * 0.5d;
            double fracBodyPolar = bodyPolarAngle;
            double fracSunPolar = sunPolarAngle;
            if (bodyPolarAngle > Math.PI * 0.5d)
            {
                fracBodyPolar = Math.PI - fracBodyPolar;
                fracSunPolar = Math.PI - fracSunPolar;
            }
            double sunDotCorrected = (1d + Vector3.Dot(sunVector, Quaternion.AngleAxis(-45f * Mathf.Sign((float)Ship.mainBody.rotationPeriod), mainBodyUp) * Ship.upAxis)) * 0.5d;
            double sunDotNormalized = (sunDotCorrected - sunBodyMinDot) / (sunBodyMaxDot - sunBodyMinDot);
            if (double.IsNaN(sunDotNormalized))
            {
                if (sunDotCorrected > 0.5d)
                    sunDotNormalized = 1d;
                else
                    sunDotNormalized = 0d;
            }

            double diurnalRange = body.latitudeTemperatureSunMultCurve.Evaluate((float)LS.latitude);
            double latTempMod = body.latitudeTemperatureBiasCurve.Evaluate((float)LS.latitude);
            double axial = (body.orbit.trueAnomaly / RadDeg + 360.0) % 360.0;
            double axialTempMod = body.axialTemperatureSunBiasCurve.Evaluate((float)axial) + body.axialTemperatureSunMultCurve.Evaluate((float)LS.latitude);
            return latTempMod + diurnalRange * sunDotNormalized + axialTempMod;
        }

        static void Start()
        {
            distance = 0;
            Bt = 0;
        }

        void OnGUI()
        {
            if (showWindow)
                windowPos = GUILayout.Window("Fall".GetHashCode(), windowPos, DrawWindow, "FallDown");
        }

        static void FixedUpdate(out double Bt, out double distance)
        {
            distance = 0;  // vertical distance moved during burntime
            Bt = 0;  // burntime
            LS.latitude = Ship.latitude;  // acceptable in case of vertical descent
            LS.longitude = Ship.longitude; // acceptable in case of vertical descent
            LS.altitude = body.TerrainAltitude(LS.latitude, LS.longitude, true);

            double CdS = 1.0;
            double Thrust, maxT, maxFF = 0;
            GetAeroStats(Ship, out Thrust, out maxT, out maxFF, out CdS);

            double Gg, Gz, D, Tv, oldD = 0;  // gravity at ground, gravity at altitude, air density, terminal velocity, previous distance
            Gg = gravity(body.gravParameter, body.Radius + LS.altitude);
            do
            {  /* with altitude, compute air density and gravity, then terminalVelocity; burntime is what required for forces on craft (thrust, drag, - gravity) to stop it.
                  at terminal velocity, drag deceleration = gravity; at 0 speed, drag deceleration is nil; drag +gravity can then be averaged as 1/2 gravity;
                  the integral of acceleration from drag (being it parabolic with speed^2) = 1/3 Gz* Bt (if at terminal velocity) (derives from http://www.intmath.com/blog/mathematics/archimedes-and-the-area-of-a-parabolic-segment-1652)
                  NOTE: vessel under drag and gravity accelerates towards the terminal velocity, but never reaches it! Therefore, drag deceleration != gravity and speed != terminalVelocity
                 */
                Gz = gravity(body.gravParameter, body.Radius + LS.altitude + distance);
                D = density(body.GetPressure(LS.altitude + distance), body.GetTemperature(LS.altitude + distance) + atmTempOffset(), 
                    PhysicsGlobals.IdealGasConstant / body.atmosphereMolarMass);
                Tv = Math.Sqrt(2 * (Ship.totalMass-maxFF*Bt) * Gg / (D * CdS));
                Bt = Tv / (maxT/2*(1/Ship.totalMass +1/(Ship.totalMass- maxFF*Bt)) + Gz/3 - (Gz+Gg)/2);
                oldD = distance;
                distance = 0.5 * (maxT - Gg) * Bt * Bt;  // note: only valid for vertical descents with thrust oriented against gravity; would require vector sum of maxT, G to compute vector distance
            } while (Math.Abs(distance-oldD) > 0.1);
        }

        void DrawWindow(int windowID)
        {
            // Enable closing of the window with "x"
            GUIStyle buttonStyle = new GUIStyle(GUI.skin.button);
            buttonStyle.padding = new RectOffset(5, 5, 3, 0);
            buttonStyle.margin = new RectOffset(1, 1, 1, 1);
            buttonStyle.stretchWidth = false;
            buttonStyle.stretchHeight = false;
            GUIStyle labelStyle = new GUIStyle(GUI.skin.label);
            labelStyle.wordWrap = false;

            GUILayout.BeginVertical();

            GUILayout.FlexibleSpace();

            GUILayout.BeginHorizontal();
            if (GUILayout.Button("X", buttonStyle))
            {
                showWindow = false;
            }
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("BurnTime: " + (Bt).ToString("N3") + " s", labelStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("distance: " + distance.ToString("N2") + " m", labelStyle);
            GUILayout.EndHorizontal();

            GUILayout.EndVertical();
            GUI.DragWindow();
        }
    }
}
