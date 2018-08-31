﻿/*
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

using Itinero.Algorithms;
using Itinero.Algorithms.Weights;
using Itinero.Data.Network;
using Itinero.Profiles;
using System;
using System.Collections.Generic;

namespace Itinero
{
    /// <summary>
    /// The base-class for generic routing functionality.
    /// </summary>
    public abstract class RouterBase
    {
        /// <summary>
        /// Gets the db.
        /// </summary>
        public abstract RouterDb Db
        {
            get;
        }

        /// <summary>
        /// Gets the Road Closure list.
        /// </summary>
        public abstract List<uint> Closures
        {
            get;
        }

        /// <summary>
        /// Gets or sets the profile factor and speed cache.
        /// </summary>
        public ProfileFactorAndSpeedCache ProfileFactorAndSpeedCache { get; set; }

        /// <summary>
        /// Flag to check all resolved points if stopping at the resolved location is possible.
        /// </summary>
        public bool VerifyAllStoppable { get; set; }

        /// <summary>
        /// Find nearest network edge independent of Profile or Closures
        /// </summary>
        public abstract Result<uint> NearestEdge(float latitude, float longitude,
            float searchDistanceInMeter = Constants.SearchDistanceInMeter);

        /// <summary>
        /// Close (or Open) road based on geographic location
        /// </summary>
        public abstract Result<RouterPoint> CloseRoad(float latitude, float longitude, bool doClose,
            float searchDistanceInMeter = Constants.SearchDistanceInMeter);

        /// <summary>
        /// Close (or Open) road based on its internal Edge ID
        /// </summary>
        public abstract Result<bool> CloseRoad(uint edgeId, bool doClose);

        /// <summary>
        /// Searches for the closest point on the routing network that's routable for the given profiles.
        /// </summary>
        /// <returns></returns>
        public abstract Result<RouterPoint> TryResolve(IProfileInstance[] profiles, float latitude, float longitude,
            Func<RoutingEdge, bool> isBetter, float searchDistanceInMeter = Constants.SearchDistanceInMeter);

        /// <summary>
        /// Checks if the given point is connected to the rest of the network. Use this to detect points on routing islands.
        /// </summary>
        /// <param name="radiusInMeter">The radius metric, that's always a distance.</param>
        /// <returns></returns>
        public abstract Result<bool> TryCheckConnectivity(IProfileInstance profile, RouterPoint point, float radiusInMeter, bool? forward = null);

        /// <summary>
        /// Calculates a route between the two locations.
        /// </summary>
        /// <returns></returns>
        public abstract Result<EdgePath<T>> TryCalculateRaw<T>(IProfileInstance profile, WeightHandler<T> weightHandler, RouterPoint source, RouterPoint target,
            RoutingSettings<T> settings = null) where T : struct;

        /// <summary>
        /// Calculates a route between the two directed edges. The route starts in the direction of the edge and ends with an arrive in the direction of the target edge.
        /// </summary>
        /// <returns></returns>
        public abstract Result<EdgePath<T>> TryCalculateRaw<T>(IProfileInstance profile, WeightHandler<T> weightHandler, long sourceDirectedEdge, long targetDirectedEdge,
            RoutingSettings<T> settings = null) where T : struct;

        /// <summary>
        /// Calculates all routes between all sources and all targets.
        /// </summary>
        /// <returns></returns>
        public abstract Result<EdgePath<T>[][]> TryCalculateRaw<T>(IProfileInstance profile, WeightHandler<T> weightHandler, RouterPoint[] sources, RouterPoint[] targets,
            RoutingSettings<T> settings = null) where T : struct;

        /// <summary>
        /// Calculates all weights between all sources and all targets.
        /// </summary>
        /// <returns></returns>
        public abstract Result<T[][]> TryCalculateWeight<T>(IProfileInstance profile, WeightHandler<T> weightHandler, RouterPoint[] sources, RouterPoint[] targets,
            ISet<int> invalidSources, ISet<int> invalidTargets, RoutingSettings<T> settings = null) where T : struct;
        
        /// <summary>
        /// Builds a route based on a raw path.
        /// </summary>
        public abstract Result<Route> BuildRoute<T>(IProfileInstance profile, WeightHandler<T> weightHandler, RouterPoint source, RouterPoint target, EdgePath<T> path) where T : struct;
    }
}