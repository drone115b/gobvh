package gobvh // import "github.com/drone115b/gobvh"

# GoBVH: Bounding Volume Hierarchy for Spatial Search in Go.

Roughly modeled after Xtrees.

Features:

    - Arbitrary dimensions
    - Arbitrary operations (nearest neighbor, raytracing, collision detection)
    - Fully dynamic (insertions, deletions, searches can be interleaved)

You might have to write some glue code (e.g. concrete Bounding Volume) but I
hope you find this implementation to be extremely flexible.

The test.go file contains a sample implementation of a nearest neighbor
search in two dimensions, if you need an example for implementing your own
searches.

## LICENSE


Copyright 2023 Mayur Patel

Licensed under the Apache License, Version 2.0 (the "License"); you may not
use this file except in compliance with the License. You may obtain a copy
of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
License for the specific language governing permissions and limitations
under the License.

## REFERENCE

### type BVH[BoundType any] struct 
{

	// Has unexported fields.

}

    BVH is the main bounding volume hierarchy object, instanced with a
    BoundType.

    Use the New() function to create one.

### func New[BoundType any](boundtraits BoundTraits[BoundType]) *BVH[BoundType]
    New(traits) returns a pointer to a new bounding volume hierarchy data
    structure.

    Please supply traits so that the bvh knows how to use the BoundType.

### func (bvh *BVH[BoundType]) Erase(element Boundable[BoundType]) bool
    BVH.Erase(element) removes a Boundable object from the data structure.

    It returns a boolean to indicate whether or not the erasure actually
    occurred.

### func (bvh *BVH[BoundType]) FindAll(s Searcher[BoundType]) error
    BVH.FindAll(searcher) is one method of search.

    It is useful when we want all elements intersecting a particular area. For
    example, in collision detection, we want all the elements that intersect
    with the bounds of the collision object.

    Contrast this with nearest neighbor search (which would more likely use
    FindNearest()). As the search progresses, the region of interest for the
    search will shrink; we would want to focus attention to the local area
    around the target first to optimize the search, so FindNearest() is more
    appropriate.

### func (bvh *BVH[BoundType]) FindNearest(s Searcher[BoundType], here BoundType) error
    BVH.FindNearest(searcher, here) is another method of search.

    It is useful when we want to focus the search around a local area. For
    example, in nearest neighbor search, the region of interest for the search
    will shrink as better matches are found during the search. As a result, we'd
    like to start the search as close to a target location as possible.

    Contrast this with collision detection, where the order of evaluation
    doesn't matter; in that case, FindAll() would be a better choice.

### func (bvh *BVH[BoundType]) ForEach(crawler BVHCrawler[BoundType]) error
    BVH.ForEach(crawler) is used to iterate over the contents of the data
    structure.

    You will need to implement a concrete struct for your crawler, to perform
    your actions.

### func (bvh *BVH[BoundType]) GetBound() BoundType
    BVH.GetBound() reports the bound for the entire data structure.

### func (bvh *BVH[BoundType]) Insert(element Boundable[BoundType])
    BVH.Insert(element) puts a Boundable object into the data structure.

    I anticipate you'll want to store pointers to concrete objects, not the
    objects themselves.

### type BVHCrawler[BoundType any] interface 
{

	BeginBound(b BoundType) error
	EndBound(b BoundType) error
	Evaluate(element Boundable[BoundType]) error

}

    BVHCrawler allows you to iterate over the data structure.

    When entering a new bounding volume, crawler.BeginBound(bound) is called.
    For each element in that volume, crawler.Evaluate(element) is called. When
    all the elements in the current volume are evaluated,
    crawler.EndBound(bound) is called. This process will repeat for each
    bounding volume in the hierarchy that contains elements.

### type BoundTraits[BoundType any] interface 
{

	IntervalRange(bound BoundType, dim uint) (float64, float64)
	Union(a BoundType, b BoundType) BoundType
	Dimensions(BoundType) uint

}

    BoundTraits is the glue code between the data structure and your
    implementation of a bounding volume. It provides to the data structure the
    operations it will require to maintain itself.

    IntervalRange(bound, dimension) is useful to convert a non-axis-aligned-box
    volume to an axis-aligned box volume. For the given dimension, please return
    the minimum and maximum extent of the bound (in that order).

    Union(bound, bound) gives the combined bounding volume of two input bounds.

    Dimensions(bound) reports on the number of dimensions for this kind of
    bound.

### type Boundable[BoundType any] interface 
{

	GetBound() BoundType

}

    Boundable is an interface for an element, which can be contained in the
    bounding volume hierarchy.

    Contained objects must be able to produce a bounding volume for themselves
    through GetBound().

### type Searcher[BoundType any] interface 
{

	DoesIntersect(bound BoundType) bool
	Evaluate(element Boundable[BoundType]) error

}

    Searcher is the interface for implementing spatial queries.

    DoesIntersect(bound) determines whether the given bound is of interest to
    the searcher and should be processed.

    Evaluate(element) is used to process a specific element. If you wish to
    support concurrency, I recommend you use goroutines within the
    searcher.Evaluate() function so that elements may be evaluated in parallel.

    The searcher will need to store local information. For example, for nearest
    neighbor search, you'll probably need the closest element found so far, the
    target position to which you want to find the nearest neighbor, and it might
    be helpful to record the closest distance between them. That closest
    distance will be used in the DoesIntersect() command, becoming a smaller and
    smaller value as the search progresses. As such, the searcher will need to
    be "reset" between searches.

