#  Itinero

**Itinero** is a routing project for .NET/Mono to calculate routes in a road network. By default the routing network is based on OpenStreetMap (OSM) but it's possible to load any road network. The most important features:

- Calculating routes from A->B.
- Calculating distance/time matrices between a set of locations.
- Processing OSM-data into a routable network.
- Generating turn-by-turn instructions.
- Routing on mobile devices and lower-resource environments.

### Changes in this fork

Updates in this fork include my prototype of on-the-fly Road Closures for routing based on dynamically changing conditions, without having to regenerate the network.  This technique can only be applied to non-Contracted networks, so it is not the fastest possible mechanism (for that, you'll have to wait for Customized Contracted Heirarchies to be implemented), but for smaller street networks, e.g. cities or counties, it is an acceptable alternative.
