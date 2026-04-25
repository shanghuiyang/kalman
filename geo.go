package kalman

import "math"

// Point represents a 2-D coordinate in longitude / latitude.
type Point struct{ Lon, Lat float64 }

// Dist returns the Euclidean distance between two points in degree-space.
func Dist(a, b Point) float64 {
	dx, dy := a.Lon-b.Lon, a.Lat-b.Lat
	return math.Sqrt(dx*dx + dy*dy)
}

func closestOnSegment(p, a, b Point) (Point, float64) {
	dx, dy := b.Lon-a.Lon, b.Lat-a.Lat
	len2 := dx*dx + dy*dy
	if len2 == 0 {
		return a, Dist(p, a)
	}
	t := ((p.Lon-a.Lon)*dx + (p.Lat-a.Lat)*dy) / len2
	t = math.Max(0, math.Min(1, t))
	cp := Point{a.Lon + t*dx, a.Lat + t*dy}
	return cp, Dist(p, cp)
}

// SnapToRoute returns the closest point on the polyline, the unit direction
// vector along the route at that point, and the segment index.
func SnapToRoute(p Point, route []Point) (snap Point, dir Point, segIdx int) {
	best := math.MaxFloat64
	for i := 0; i < len(route)-1; i++ {
		cp, d := closestOnSegment(p, route[i], route[i+1])
		if d < best {
			best = d
			snap = cp
			segIdx = i
		}
	}
	dx := route[segIdx+1].Lon - route[segIdx].Lon
	dy := route[segIdx+1].Lat - route[segIdx].Lat
	norm := math.Sqrt(dx*dx + dy*dy)
	if norm > 0 {
		dir = Point{dx / norm, dy / norm}
	}
	return
}
