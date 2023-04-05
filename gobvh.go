// # GoBVH: Bounding Volume Hierarchy for Spatial Search in Go.
//
// Roughly modeled after Xtrees.
//
// Features:
//  - Arbitrary dimensions
//  - Arbitrary operations (nearest neighbor, raytracing, collision detection)
//  - Fully dynamic (insertions, deletions, searches can be interleaved)
//
// You might have to write some glue code (e.g. concrete Bounding Volume)
// but I hope you find this implementation to be extremely flexible.
//
// The test.go file contains a sample implementation of a nearest neighbor
// search in two dimensions, if you need an example for implementing your own
// searches.
//
// ## LICENSE
//
// Copyright 2023 Mayur Patel
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
package gobvh

import (
	"math" // min(), max()
)

// ==============================================

//
// BoundTraits is the glue code between the data structure
// and your implementation of a bounding volume.  It provides
// to the data structure the operations it will require to maintain itself.
//
// IntervalRange(bound, dimension) is useful to convert a non-axis-aligned-box volume to
// an axis-aligned box volume.  For the given dimension, please return the minimum
// and maximum extent of the bound (in that order).
//
// Union(bound, bound) gives the combined bounding volume of two input bounds.
//
// Dimensions(bound) reports on the number of dimensions for this kind of bound.
//
type BoundTraits[BoundType any] interface {
	IntervalRange(bound BoundType, dim uint) (float64, float64)
	Union(a BoundType, b BoundType) BoundType
	Dimensions(BoundType) uint
}

//
// Boundable is an interface for an element, which can be contained
// in the bounding volume hierarchy.
//
// Contained objects must be able to produce a bounding volume for themselves
// through GetBound().
//
type Boundable[BoundType any] interface {
	GetBound() BoundType
}

//
// Searcher is the interface for implementing spatial queries.
//
// DoesIntersect(bound) determines whether the given bound is of interest to
// the searcher and should be processed.
//
// Evaluate(element) is used to process a specific element.
// If you wish to support concurrency, I recommend you use goroutines within the
// searcher.Evaluate() function so that elements may be evaluated in parallel.
//
// The searcher will need to store local information.  For example, for nearest
// neighbor search, you'll probably need the closest element found so far,
// the target position to which you want to find the nearest neighbor, and
// it might be helpful to record the closest distance between them.
// That closest distance will be used in the DoesIntersect() command, becoming
// a smaller and smaller value as the search progresses.  As such, the searcher
// will need to be "reset" between searches.
//
type Searcher[BoundType any] interface {
	DoesIntersect(bound BoundType) bool
	Evaluate(element Boundable[BoundType]) error
}

// ==============================================

//
// BVH is the main bounding volume hierarchy object, instanced with a BoundType.
//
// Use the New() function to create one.
//
type BVH[BoundType any] struct {
	root        bvhNode[BoundType]
	boundtraits BoundTraits[BoundType]
}

// ..............................................

//
// BVH.GetBound() reports the bound for the entire data structure.
//
func (bvh *BVH[BoundType]) GetBound() BoundType {
	return bvh.root.bound
}

// ..............................................

//
// New(traits) returns a pointer to a new bounding volume hierarchy data structure.
//
// Please supply traits so that the bvh knows how to use the BoundType.
//
func New[BoundType any](boundtraits BoundTraits[BoundType]) *BVH[BoundType] {
	return &BVH[BoundType]{
		boundtraits: boundtraits,
	}
}

// ..............................................

//
// BVH.FindAll(searcher) is one method of search.
//
// It is useful when
// we want all elements intersecting a particular area.  For example,
// in collision detection, we want all the elements that intersect
// with the bounds of the collision object.
//
// Contrast this with nearest
// neighbor search (which would more likely use FindNearest()).
// As the search progresses, the region of interest for the search will
// shrink; we would want to focus attention to the local area around the
// target first to optimize the search, so FindNearest() is more appropriate.
//
func (bvh *BVH[BoundType]) FindAll(s Searcher[BoundType]) error {
	var err error = nil
	if len(bvh.root.children) > 0 {
		err = findDown(s, &bvh.root, nil)
	}
	return err
}

// ..............................................

//
// BVH.FindNearest(searcher, here) is another method of search.
//
// It is useful when
// we want to focus the search around a local area.
// For example, in nearest neighbor search, the region of interest for the search
// will shrink as better matches are found during the search.  As a result,
// we'd like to start the search as close to a target location as possible.
//
// Contrast this with collision detection, where the order of evaluation
// doesn't matter; in that case, FindAll() would be a better choice.
//
func (bvh *BVH[BoundType]) FindNearest(s Searcher[BoundType], here BoundType) error {
	// start at the leaf of the hierarchy:
	lastnode := chooseLeaf(bvh, here)

	// move up from the bottom:
	return findUp(s, lastnode, nil)
}

// ..............................................

//
// BVH.Insert(element) puts a Boundable object into the data structure.
//
// I anticipate you'll want to store pointers to concrete
// objects, not the objects themselves.
//
func (bvh *BVH[BoundType]) Insert(element Boundable[BoundType]) {
	elembound := element.GetBound()

	if len(bvh.root.children) == 0 {
		// first insertion is a special case:
		bvh.root.children = append(bvh.root.children, element)
		bvh.root.bound = elembound

	} else {

		// find appropriate leaf and insert it there:
		chosen := chooseLeaf(bvh, elembound)
		chosen.children = append(chosen.children, element)
		chosen.bound = (*bvh).boundtraits.Union(chosen.bound, elembound)

		// update ancestors' bounds:
		updatenode := chosen.parent
		for updatenode != nil {
			(*updatenode).bound = bvh.boundtraits.Union((*updatenode).bound, elembound)
			updatenode = updatenode.parent
		}

		splitNode(bvh.boundtraits, chosen, &bvh.root)
	} // end if insert into non-root

	return
}

// ..............................................

//
// BVH.Erase(element) removes a Boundable object from the data structure.
//
// It returns a boolean to indicate whether or not the erasure actually occurred.
//
func (bvh *BVH[BoundType]) Erase(element Boundable[BoundType]) bool {
  diderase, erasenode := eraseChild(bvh.boundtraits, &bvh.root, element, element.GetBound())
	for erasenode != nil {
		eraseparent := erasenode.parent
		if eraseparent != nil && len(erasenode.children) == 0 {
			var toerase Boundable[BoundType] = erasenode
			eraseChild(bvh.boundtraits, eraseparent, toerase, toerase.GetBound())
		} else {
			break
		}
		erasenode = eraseparent
	}
	return diderase
}

// ==============================================

//
// BVHCrawler allows you to iterate over the data structure.
//
// When entering a new bounding volume, crawler.BeginBound(bound) is called.
// For each element in that volume, crawler.Evaluate(element) is called.
// When all the elements in the current volume are evaluated, crawler.EndBound(bound) is called.
// This process will repeat for each bounding volume in the hierarchy
// that contains elements.
//
type BVHCrawler[BoundType any] interface {
	BeginBound(b BoundType) error
	EndBound(b BoundType) error
	Evaluate(element Boundable[BoundType]) error
}

// ..............................................

//
// BVH.ForEach(crawler) is used to iterate over the contents of the
// data structure.
//
// You will need to implement a concrete struct for
// your crawler, to perform your actions.
//
func (bvh *BVH[BoundType]) ForEach(crawler BVHCrawler[BoundType]) error {
	return forEachNode(crawler, &bvh.root)
}

// ..............................................

func forEachNode[BoundType any](crawler BVHCrawler[BoundType], node *bvhNode[BoundType]) error {
	if node != nil {
		var err error
		var crawlhere bool = false

		for _, child := range node.children {
			if child != nil {
				value, ok := child.(*bvhNode[BoundType])
				if ok {
					err = forEachNode(crawler, value)
					if err != nil {
						return err
					}
				} else {
					crawlhere = true
				}
			}
		} // end for

		if crawlhere {
			err = crawler.BeginBound(node.bound)
			if err != nil {
				return err
			}

			for _, child := range node.children {
				if child != nil {
					_, ok := child.(*bvhNode[BoundType])
					if !ok {
						err = crawler.Evaluate(child)
						if err != nil {
							return err
						}
					}
				}

			} // end for

			err = crawler.EndBound(node.bound)
			if err != nil {
				return err
			}
		}

	}

	return nil
}

// ==============================================

type bvhNode[BoundType any] struct {
	bound    BoundType
	children []Boundable[BoundType]
	parent   *bvhNode[BoundType]
}

// ..............................................

// makes bvhNode a Boundable[BoundType]:
func (node *bvhNode[BoundType]) GetBound() BoundType {
	return node.bound
}

// ..............................................

func findUp[BoundType any](s Searcher[BoundType], node *bvhNode[BoundType], skip *bvhNode[BoundType]) error {
	if node != nil {
		err := findDown(s, node, skip)
		if err != nil {
			return err
		}
		err = findUp(s, node.parent, node)
		if err != nil {
			return err
		}
	}
	return nil
}

// ..............................................

func findDown[BoundType any](s Searcher[BoundType], node *bvhNode[BoundType], skip *bvhNode[BoundType]) error {
	if node != nil {
		var err error
		if s.DoesIntersect(node.GetBound()) {
			for _, child := range node.children {
				if child != nil {
					value, ok := child.(*bvhNode[BoundType])
					if ok {
						if value != skip {
							err = findDown(s, value, skip)
						}
					} else {
						err = s.Evaluate(child)
					}
				}
				if err != nil {
					return err
				}
			}
		}
	}
	return nil
}

// ..............................................

// from the given node, select the immediate child "closest" to the given bound, b
func chooseChild[BoundType any](bounder BoundTraits[BoundType], node *bvhNode[BoundType], b BoundType) *bvhNode[BoundType] {
	choosemetric := 1e38
	var chosen *bvhNode[BoundType] = nil
	if node != nil {
		for _, child := range node.children {
			value, ok := child.(*bvhNode[BoundType])
			if ok {
				_, metric := furthestDistanceMetric(bounder, (*value).GetBound(), b)
				if metric < choosemetric {
					choosemetric = metric
					chosen = value
				}
			} // if node type
		} // end for
	}
	return chosen
}

// return the leaf of "tree" closest to b.  This isn't the element, this is the node containing elements.
func chooseLeaf[BoundType any](tree *BVH[BoundType], b BoundType) *bvhNode[BoundType] {
	node := &tree.root
	lastnode := &tree.root
	for node != nil {
		chosen := chooseChild(tree.boundtraits, node, b)
		lastnode = node
		node = chosen
	} // end for
	return lastnode
}

// ..............................................

// erase node from subtree rooted at parent; and update parent and all other ancestor bounds.
func eraseChild[BoundType any](bounder BoundTraits[BoundType], parent *bvhNode[BoundType], element Boundable[BoundType], elembound BoundType) (bool, *bvhNode[BoundType]) {
	erased := false
	erasedhere := false
	var container *bvhNode[BoundType]

	if parent != nil {
		doesintersect, _ := furthestDistanceMetric(bounder, elembound, parent.bound)
		if doesintersect {

			for index, child := range parent.children {
				value, ok := child.(*bvhNode[BoundType])
				if ok {
					erased, container = eraseChild(bounder, value, element, elembound)
					if erased {
						break // for
					}
				}

				if child == element {
					// erase node from parent.children slice
					parent.children[index] = parent.children[len(parent.children)-1]
					parent.children = parent.children[:len(parent.children)-1]
					container = parent
					erasedhere = true
					break // for
				} // if child is element
			} // end for

			if true == erasedhere {
				updatenode := container
				for updatenode != nil {
					recalculateBounds(bounder, updatenode)
					updatenode = updatenode.parent
				} // end for update ancestors' bounds
			} // if erased here
		} // if node bound intersects element bound
	} // if parent

	return erased || erasedhere, container
}

// ..............................................

func recalculateBounds[BoundType any](bounder BoundTraits[BoundType], node *bvhNode[BoundType]) {
	initialized := false
	for _, child := range node.children {
		if initialized {
			node.bound = bounder.Union(child.GetBound(), node.bound)
		} else {
			initialized = true
			node.bound = child.GetBound()
		}
	}
}

// ..............................................

func fixParentPointers[BoundType any](node *bvhNode[BoundType]){
	if node != nil {
		for _, child := range(node.children) {
			childnode, ok := child.(*bvhNode[BoundType])
			if ok {
				childnode.parent = node
			}
		}
  }
}

// ..............................................

//
func splitNode[BoundType any](bounder BoundTraits[BoundType], node *bvhNode[BoundType], root *bvhNode[BoundType]) {
	parent := node
	for parent != nil && len(parent.children)%16 == 0 && len(parent.children) > 0 {
		if root == parent {
			// splitting the root is a special case
			// move root children to new node:
			newnode := bvhNode[BoundType]{
				children: root.children[:],
				parent:   root,
				bound:    root.bound,
			}
			// fix parent pointers for moved children:
			fixParentPointers(&newnode)

			// make new children for root and split the new node:
			root.children = make([]Boundable[BoundType], 0, 8)
			root.children = append(root.children, &newnode)
			parent = &newnode

		} else {
			// splitting a "normal" node, not the root
		  // assert that parent.parent != nil

			// get opposing corners of the bound:
			bound0, bound1 := getSplitBounds(bounder, parent)

      // reuse node "parent" as node1, create a new node0
			node0 := &(bvhNode[BoundType]{parent: parent.parent})
			node1 := parent

      // divide children of "parent" between node0 and node1
			node0.children, node1.children = partitionSplit(bounder, parent, bound0, bound1)

			// if a minimally useful split occurred, then commit; otherwise revert:
			if len(node0.children) > 1 && len(node1.children) > 1 {
				fixParentPointers(node0)
			  parent.parent.children = append(parent.parent.children, node0)

				recalculateBounds(bounder, node0)
				recalculateBounds(bounder, node1)

			} else {
				// revert the node split:
				node1.children = append(node1.children, node0.children...)
				break // break for
			}

      parent = parent.parent
		} // end if root

	} // end for
	return
}

// ..............................................

func getSplitBounds[BoundType any](bounder BoundTraits[BoundType], node *bvhNode[BoundType]) (BoundType, BoundType) {
	var b0 BoundType
	var b1 BoundType
	var chosenmetric float64

	if len(node.children) > 0 {
		b0 = node.children[0].GetBound()
	}
	if len(node.children) > 1 {
		b1 = node.children[1].GetBound()
		_, chosenmetric = furthestDistanceMetric(bounder, b0, b1)
	}
	for index := 2; index < len(node.children); index++ {
		thisbound := node.children[index].GetBound()
		_, metric0 := furthestDistanceMetric(bounder, thisbound, b1)
		_, metric1 := furthestDistanceMetric(bounder, thisbound, b0)

		if metric0 > metric1 && metric0 > chosenmetric {
			chosenmetric = metric0
			b0 = thisbound
		}

		if metric1 > metric0 && metric1 > chosenmetric {
			chosenmetric = metric1
			b1 = thisbound
		}

	} // end for
	return b0, b1
}

// ..............................................

// returns two slices, each contains elements proximate to either bound0 or bound1 respectfully.
func partitionSplit[BoundType any](bounder BoundTraits[BoundType], node *bvhNode[BoundType], bound0 BoundType, bound1 BoundType) ([]Boundable[BoundType], []Boundable[BoundType]) {
	store0 := make([]Boundable[BoundType], 0, 8)
	store1 := make([]Boundable[BoundType], 0, 8)

	for _, child := range node.children {
		thisbound := child.GetBound()
		_, metric0 := furthestDistanceMetric(bounder, thisbound, bound0)
		_, metric1 := furthestDistanceMetric(bounder, thisbound, bound1)
		if metric0 < metric1 {
			store0 = append(store0, child)
		} else {
			store1 = append(store1, child)
		}
	} // end for
	return store0, store1
}

// ==============================================

// To maximize overlap between the bounding volumes, minimize this metric
// from "Similarity metrics for bounding volumes", SIGGRAPH '07: ACM SIGGRAPH 2007 posters
// This uses the L1 metric which scales well to high dimensions
func furthestDistanceMetric[BoundType any](bounder BoundTraits[BoundType], first BoundType, second BoundType) (bool, float64) {
	var metric float64 = 0.0
	doesintersect := false
	var i uint
	for i = 0; i < bounder.Dimensions(first); i++ {
		lo0, hi0 := bounder.IntervalRange(first, i)
		lo1, hi1 := bounder.IntervalRange(second, i)

		var n float64 = math.Min(hi0 - lo1, hi1 - lo0)
		var m float64 = math.Max(hi0 - lo1, hi1 - lo0)

		if n >= 0.0 {
			doesintersect = true
		}
		metric += m
	}
	return doesintersect, metric
}
