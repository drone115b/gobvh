package gobvh

import (
	"fmt"
	"math"
	"testing"
)

// ========================================================
// Basic 2D types:

type Point2D [2]float64

// make Point2D a Boundable[AABB2D]
func (p Point2D) GetBound() AABB2D {
	return AABB2D{p, p}
}

// ........................................................

// BoundType for our application:
type AABB2D struct {
	L Point2D
	H Point2D
}

// ........................................................

func distance2D(a Point2D, b Point2D) float64 {
	return math.Sqrt((a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1]))
}

func distancePointBox2D(a Point2D, aabb AABB2D) (bool, float64) {
	var dist float64 = 0.0
	var n float64
	var i uint
	doesintersect := true
	for i = 0; i < 2; i++ {
		n = math.Min(a[i]-aabb.L[i], aabb.H[i]-a[i])
		doesintersect = doesintersect && (n >= 0.0)
		dist += math.Min(0.0, n) * math.Min(0.0, n)
	}
	return doesintersect, math.Sqrt(dist)
}

// ========================================================

// Searcher implementation for our nearest neighbor search in 2D:
type NearestNeighbor2D struct {
	Target        Point2D
	Found         Boundable[AABB2D]
	FoundDistance float64
	t             *testing.T
}

func (nn *NearestNeighbor2D) DoesIntersect(aabb AABB2D) bool {
	_, dist := distancePointBox2D(nn.Target, aabb)
	return dist <= nn.FoundDistance
}

func (nn *NearestNeighbor2D) Evaluate(element Boundable[AABB2D]) error {
	point, ok := element.(Point2D)
	if ok {
		dist := distance2D(nn.Target, point)
		if nn.Found == nil || nn.FoundDistance > dist {
			nn.FoundDistance = dist
			nn.Found = element
		}
	} else {
		return fmt.Errorf("Unexpected type in BVH (%T), not a Point2D.", element)
	}
	return nil
}

// ........................................................

// BoundTraits[AABB2D] implementation:
type Traits2D struct{}

func (bounder Traits2D) IntervalRange(bound AABB2D, dim uint) (float64, float64) {
	return bound.L[dim], bound.H[dim]
}

func (bounder Traits2D) Union(a AABB2D, b AABB2D) AABB2D {
	var result AABB2D
	result.L[0] = math.Min(a.L[0], b.L[0])
	result.H[0] = math.Max(a.H[0], b.H[0])
	result.L[1] = math.Min(a.L[1], b.L[1])
	result.H[1] = math.Max(a.H[1], b.H[1])
	return result
}

func (bounder Traits2D) Dimensions(aabb AABB2D) uint {
	return 2
}

// ========================================================

func visualize(t *testing.T, node *bvhNode[AABB2D], indent string) {
	t.Logf("%s(%f, %f)-(%f, %f)  |  %d children\n", indent, node.bound.L[0], node.bound.L[1], node.bound.H[0], node.bound.H[1], len(node.children))
	for _, child := range node.children {
		childnode, ok := child.(*bvhNode[AABB2D])
		if ok {
			visualize(t, childnode, indent+"  ")
			if childnode.parent != node {
				t.Errorf("Bad parent pointer found on child node.\n")
			}
		}
	}
}

// ========================================================

func simpleNNSearch(t *testing.T, bvh *BVH[AABB2D], target Point2D, expected Point2D, nearest bool) Boundable[AABB2D] {
	// simple search:
	searcher := NearestNeighbor2D{}
	searcher.FoundDistance = 1e38 // indicator that no match is found
	searcher.Target = target
	searcher.t = t

	var err error = nil
	if nearest {
		err = bvh.FindNearest(&searcher, searcher.Target.GetBound())
	} else {
		err = bvh.FindAll(&searcher)
	}
	if err != nil {
		t.Errorf(err.Error())
	}
	foundpoint, ok := searcher.Found.(Point2D)
	if ok {
		if foundpoint[0] != expected[0] || foundpoint[1] != expected[1] {
			t.Errorf("Closest point expected (%f, %f) but search returned (%f, %f)", expected[0], expected[1], foundpoint[0], foundpoint[1])
		}
	} else {
		t.Errorf("Unexpected element type (%T{%v}) returned from nearest neighbor search", searcher.Found, searcher.Found)
	}

	return searcher.Found
}

// ========================================================

//
// Simple crawler to verify element bound against its container:
//
type CheckBound struct {
	bound AABB2D
	T     *testing.T
}

func (cb *CheckBound) BeginBound(b AABB2D) error {
	cb.bound = b
	return nil
}

func (cb *CheckBound) EndBound(b AABB2D) error {
	return nil
}

func (cb *CheckBound) Evaluate(element Boundable[AABB2D]) error {
	elembound := element.GetBound()
	if elembound.L[0] < cb.bound.L[0] || elembound.L[1] < cb.bound.L[1] || elembound.H[0] > cb.bound.H[0] || elembound.H[1] > cb.bound.H[1] {
		if cb.T != nil {
			cb.T.Errorf("element (%v) exceeds bound of container: (%f %f)-(%f %f)", element, cb.bound.L[0], cb.bound.L[1], cb.bound.H[0], cb.bound.H[1])
		}
		return fmt.Errorf("element (%v) exceeds bound of container: (%f %f)-(%f %f)", element, cb.bound.L[0], cb.bound.L[1], cb.bound.H[0], cb.bound.H[1])
	}
	return nil
}

// ========================================================

func TestBVHNearestNeighbor2D(t *testing.T) {
	var x, y float64

	// instance our bounding volume hierarchy
	var bounder BoundTraits[AABB2D]
	bounder = Traits2D{}
	bvh := New(bounder)

	// insert integer lattice corners
	t.Logf("Insert (0 0)-(31 31):\n")
	for x = 0.0; x < 32.0; x += 1.0 {
		for y = 0.0; y < 32.0; y += 1.0 {
			bvh.Insert(Point2D{x, y})
		}
	}
	// search lattice points
	for x = 0.0; x < 8.0; x += 1.0 {
		for y = 0.0; y < 8.0; y += 1.0 {
			simpleNNSearch(t, bvh, Point2D{x + 0.1, y - 0.1}, Point2D{x, y}, true)    // bvh.FindNearest()
			simpleNNSearch(t, bvh, Point2D{x - 0.15, y + 0.15}, Point2D{x, y}, false) // bvh.FindAll()
		}
	}
	t.Logf("Visualization:\n")
	visualize(t, &(bvh.root), "  ")

	// Check bound:
	var cb CheckBound
	cb.T = t
	bvh.ForEach(&cb)

	// Erase stuff
	t.Logf("Erase (1 0)-(31 31):\n")
	for x = 1.0; x < 32.0; x += 1.0 {
		for y = 0.0; y < 32.0; y += 1.0 {
			found := simpleNNSearch(t, bvh, Point2D{x + 0.1, y + 0.1}, Point2D{x, y}, true)
			bvh.Erase(found)
		}
	}
	for y = 0.0; y < 32.0; y += 1.0 {
		simpleNNSearch(t, bvh, Point2D{0.1, y - 0.1}, Point2D{0.0, y}, true)    // bvh.FindNearest()
		simpleNNSearch(t, bvh, Point2D{0.15, y + 0.15}, Point2D{0.0, y}, false) // bvh.FindAll()
	}
	bvh.ForEach(&cb) // verify bounds again
	t.Logf("Visualization:\n")
	visualize(t, &(bvh.root), "  ")

	t.Logf("Erase all remaining elements")
	for y = 0.0; y < 32.0; y += 1.0 {
		found := simpleNNSearch(t, bvh, Point2D{0.0, y + 0.1}, Point2D{0.0, y}, true)
		bvh.Erase(found)
	}
	if 0 != len(bvh.root.children) {
		t.Errorf("Expected empty tree, but detecting %d elements", len(bvh.root.children))
	}
	bvh.ForEach(&cb) // verify bounds again
	t.Logf("Visualization:\n")
	visualize(t, &(bvh.root), "  ")

	t.Logf("Insert (0, 0)-(0 31)")
	for y = 0.0; y < 32.0; y += 1.0 {
		bvh.Insert(Point2D{0.0, y})
	}
	bvh.ForEach(&cb) // verify bounds again
	t.Logf("Visualization:\n")
	visualize(t, &(bvh.root), "  ")

	t.Logf("Erase all remaining elements")
	for y = 0.0; y < 32.0; y += 1.0 {
		found := simpleNNSearch(t, bvh, Point2D{0.0, y + 0.1}, Point2D{0.0, y}, true)
		bvh.Erase(found)
	}
	bvh.ForEach(&cb) // verify bounds again
	t.Logf("Visualization:\n")
	visualize(t, &(bvh.root), "  ")

	return
}
