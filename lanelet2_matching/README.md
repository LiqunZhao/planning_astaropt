# Lanelet2 matching

The matching modules for lanelet2. This modules provides functions to determine in which lanelet an object is currently located.

## Matching functions

* **Deterministic matching**: Find all lanelets to which an object has less than a certain distance.
* **Probabilistic matching**: Compute the squared Mahalanobis distance of the object pose to the lanelet to reason about the probability of tha match, as suggested by Petrich et al. ([DOI:0.1109/ITSC.2013.6728549](https://doi.org/10.1109/ITSC.2013.6728549))


