/*
 *  Licensed to SharpSoftware under one or more contributor
 *  license agreements. See the NOTICE file distributed with this work for 
 *  additional information regarding copyright ownership.
 * 
 *  SharpSoftware licenses this file to you under the Apache License, 
 *  Version 2.0 (the "License"); you may not use this file except in 
 *  compliance with the License. You may obtain a copy of the License at
 * 
 *       http://www.apache.org/licenses/LICENSE-2.0
 * 
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

using Itinero.LocalGeo;
using System.Collections.Generic;
using System.Linq;

namespace Itinero.Navigation.Directions
{
    /// <summary>
    /// Calculates direction.
    /// </summary>
    public static class DirectionCalculator
    {
        /// <summary>
        /// Calculates the angle in randians at coordinate2.
        /// </summary>
        public static float Angle(Coordinate coordinate1, Coordinate coordinate2, Coordinate coordinate3)
        {
            var v11 = coordinate1.Latitude - coordinate2.Latitude;
            var v10 = coordinate1.Longitude - coordinate2.Longitude;

            var v21 = coordinate3.Latitude - coordinate2.Latitude;
            var v20 = coordinate3.Longitude - coordinate2.Longitude;

            var v1size = System.Math.Sqrt(v11 * v11 + v10 * v10);
            var v2size = System.Math.Sqrt(v21 * v21 + v20 * v20);

            // filter out the vectors that are parallel.
            if (v10 == v20 && 
                v11 == v21)
            {
                return 0;
            }
            else if (v10 == v20 && 
                v11 == -v21)
            {
                return (float)(System.Math.PI / 2.0f);
            }
            else if (v10 == -v20 &&
                v11 == v21)
            {
                return (float)(-System.Math.PI / 2.0f);
            }
            else if (v10 == -v20 &&
                v11 == -v21)
            {
                return (float)System.Math.PI;
            }

            var dot = (double)(v11 * v21 + v10 * v20);
            var cross = (double)(v10 * v21 - v11 * v20);

            // split per quadrant.
            double angle;
            if (dot > 0)
            { // dot > 0
                if (cross > 0)
                { // dot > 0 and cross > 0
                    // Quadrant 1
                    angle = (double)System.Math.Asin(cross / (v1size * v2size));
                    if (angle < System.Math.PI / 4f)
                    { // use cosine.
                        angle = (double)System.Math.Acos(dot / (v1size * v2size));
                    }
                    // angle is ok here for quadrant 1.
                }
                else
                { // dot > 0 and cross <= 0
                    // Quadrant 4
                    angle = (double)(System.Math.PI * 2.0f) + (double)System.Math.Asin(cross / (v1size * v2size));
                    if (angle > (double)(System.Math.PI * 2.0f) - System.Math.PI / 4f)
                    { // use cosine.
                        angle = (double)(System.Math.PI * 2.0f) - (double)System.Math.Acos(dot / (v1size * v2size));
                    }
                    // angle is ok here for quadrant 1.
                }
            }
            else
            { // dot <= 0
                if (cross > 0)
                { // dot > 0 and cross > 0
                    // Quadrant 2
                    angle = (double)System.Math.PI - (double)System.Math.Asin(cross / (v1size * v2size));
                    if (angle > System.Math.PI / 2f + System.Math.PI / 4f)
                    { // use cosine.
                        angle = (double)System.Math.Acos(dot / (v1size * v2size));
                    }
                    // angle is ok here for quadrant 2.
                }
                else
                { // dot > 0 and cross <= 0
                    // Quadrant 3
                    angle = -(-(double)System.Math.PI + (double)System.Math.Asin(cross / (v1size * v2size)));
                    if (angle < System.Math.PI + System.Math.PI / 4f)
                    { // use cosine.
                        angle = (double)(System.Math.PI * 2.0f) - (double)System.Math.Acos(dot / (v1size * v2size));
                    }
                    // angle is ok here for quadrant 3.
                }
            }
            return (float)angle;
        }

        /// <summary>
        /// Calculates the direction of one line segment relative to another.
        /// </summary>
        public static RelativeDirection Calculate(Route route, int currentShape, Coordinate coordinate1, Coordinate coordinate2, Coordinate coordinate3)
        {
            var direction = new RelativeDirection();

            var margin = 65.0;
            var straightOn = 10.0;
            var turnBack = 5.0;

            var angleRandians = DirectionCalculator.Angle(coordinate1, coordinate2, coordinate3);
            var angle = angleRandians.ToDegrees();

            angle = angle.NormalizeDegrees();

            // an inline branch is any branch between the rightmost and leftmost
            bool isInlineBranch = false;
            // only consider branches at the current 'shape'
            int currentBranches = 0;
            int startBranch = 0, endBranch = 0;

            // if there are more than 2 branches, and we're NOT turning on to the rightmost or leftmost branch,
            // then we're effectively continuing on an inline branch, so the directions should vary from the norm.

            // first, determine how many branches are at this intersection (having the same 'shape' number)
            for (int i = 0; i < route.Branches.Length; i++)
            {
                if (route.Branches[i].Shape == currentShape)
                {
                    if (currentBranches == 0)
                        startBranch = endBranch = i;
                    else
                        endBranch = i;
                    // increment count
                    currentBranches++;
                }
            }

            // we're only concerned if there is more than 1 branch here
            if (true) // currentBranches > 1)
            {
                Dictionary<int, double> orderedBranches = new System.Collections.Generic.Dictionary<int, double>();

                // for some reason, sometimes only 2 branches are returned even when there are 3;
                // and the missing branch seems to always be the chosen path; therefore, we will
                // make sure the chosen path is always in the collection so that we can make the 
                // right choice on the final direction
                bool includesChosenPath = false;

                // determine angles of each of the Branches
                for (int i = startBranch; i <= endBranch; i++)
                {
                    // additionally, there are times when one of the 'Branch' points is actually
                    // identical to one of the coordinate points, resulting in an angle of NaN;
                    // we will bypass any such 'Branch' as erroneous
                    if (coordinate1.Equals(route.Branches[i].Coordinate) || coordinate2.Equals(route.Branches[i].Coordinate)) continue;

                    // angle will use coordinate1, coordinate2, and current branch
                    var aRads = DirectionCalculator.Angle(coordinate1, coordinate2, route.Branches[i].Coordinate);
                    // additional safety check
                    if (aRads == float.NaN) continue;

                    var aDegs = aRads.ToDegrees();
                    aDegs = aDegs.NormalizeDegrees();

                    // add to dictionary (branch index, angle)
                    orderedBranches.Add(i, aDegs);

                    // although the paths are almost always within 3 decimal places,
                    // it appears that occasionally the angle difference is as much as
                    // 4 degrees, so we will allow up to 4 degrees difference in comparison
                    // (it's fair to say for now that no 2 streets could be within 4 degrees)
                    if (System.Math.Abs(aDegs - angle) < 4.0) includesChosenPath = true;
                }
                // before sorting, if chosen path is not in the dictionary, add it here
                if (!includesChosenPath) orderedBranches.Add(endBranch + 1, angle);

                // now sort by branch angles to get branches in order, right to left (min angle to max angle)
                var items = from pair in orderedBranches
                            orderby pair.Value ascending
                            select pair;
                // iterate branches in order by angle (branch index is irrelevant at this point)
                foreach (KeyValuePair<int, double> pair in items)
                {
                    double aDegs = pair.Value;
                    // is this branch the current route? (having angle equal to current street angle)
                    if (System.Math.Abs(aDegs - angle) < 4.0)
                    {
                        // we found the branch of the route; see if it is one of the inline branches (not the first or last)
                        if (!(pair.Key == items.First().Key || pair.Key == items.Last().Key))
                        {
                            isInlineBranch = true;
                            // we can stop looking
                            break;
                        }
                    }
                }
            }


            // directions as defined are valid if turning on
            // to 
            if (angle >= 360 - turnBack || angle < turnBack)                // +/- 5 deg
            {
                direction.Direction = RelativeDirectionEnum.TurnBack;
            }
            else if (angle >= turnBack && angle < 90 - margin)              // 5 to 25 deg
            {
                direction.Direction = RelativeDirectionEnum.SharpRight;
                // sharp right would imply the rightmost, so if not the rightmost, use 'Right' instead
                if (isInlineBranch) direction.Direction = RelativeDirectionEnum.Right;
            }
            else if (angle >= 90 - margin && angle < 90 + margin)           // 25 to 155 deg
            {
                direction.Direction = RelativeDirectionEnum.Right;
                // right would imply the rightmost, so if not the rightmost, use 'SlightlyRight' instead
                if (isInlineBranch) direction.Direction = RelativeDirectionEnum.SlightlyRight;
            }
            else if (angle >= 90 + margin && angle < 180 - straightOn)      // 155 to 170 deg
            {
                direction.Direction = RelativeDirectionEnum.SlightlyRight;
            }
            else if (angle >= 180 - straightOn && angle < 180 + straightOn) // 170 to 190 deg
            {
                direction.Direction = RelativeDirectionEnum.StraightOn;
            }
            else if (angle >= 180 + straightOn && angle < 270 - margin)     // 190 to 205 deg
            {
                direction.Direction = RelativeDirectionEnum.SlightlyLeft;
            }
            else if (angle >= 270 - margin && angle < 270 + margin)         // 205 to 335 deg
            {
                direction.Direction = RelativeDirectionEnum.Left;
                // left would imply the leftmost, so if not the leftmost, use 'SlightlyLeft' instead
                if (isInlineBranch) direction.Direction = RelativeDirectionEnum.SlightlyLeft;
            }
            else if (angle >= 270 + margin && angle < 360 - turnBack)       // 335 to 355 deg
            {
                direction.Direction = RelativeDirectionEnum.SharpLeft;
                // sharp left would imply the leftmost, so if not the leftmost, use 'Left' instead
                if (isInlineBranch) direction.Direction = RelativeDirectionEnum.Left;
            }
            direction.Angle = (float)angle;

            return direction;
        }

        /// <summary>
        /// Calculates the direction of a segment.
        /// </summary>
        public static DirectionEnum Calculate(Coordinate coordinate1, Coordinate coordinate2)
        {
            var angle = (double)DirectionCalculator.Angle(new Coordinate(coordinate1.Latitude + 0.01f, coordinate1.Longitude),
                coordinate1, coordinate2);

            angle = angle.ToDegrees();
            angle = angle.NormalizeDegrees();

            if (angle < 22.5 || angle >= 360 - 22.5)
            { // north
                return DirectionEnum.North;
            }
            else if (angle >= 22.5 && angle < 90 - 22.5)
            { // north-east.
                return DirectionEnum.NorthWest;
            }
            else if (angle >= 90 - 22.5 && angle < 90 + 22.5)
            { // east.
                return DirectionEnum.West;
            }
            else if (angle >= 90 + 22.5 && angle < 180 - 22.5)
            { // south-east.
                return DirectionEnum.SouthWest;
            }
            else if (angle >= 180 - 22.5 && angle < 180 + 22.5)
            { // south
                return DirectionEnum.South;
            }
            else if (angle >= 180 + 22.5 && angle < 270 - 22.5)
            { // south-west.
                return DirectionEnum.SouthEast;
            }
            else if (angle >= 270 - 22.5 && angle < 270 + 22.5)
            { // south-west.
                return DirectionEnum.East;
            }
            else if (angle >= 270 + 22.5 && angle < 360 - 22.5)
            { // south-west.
                return DirectionEnum.NorthEast;
            }
            return DirectionEnum.North;
        }
    }
}