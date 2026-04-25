package kalman

// Config holds tunable Kalman filter parameters.
type Config struct {
	// ProcessNoisePos is the position process noise variance (qp).
	// Smaller → stronger trust in the route-guided motion model.
	ProcessNoisePos float64

	// ProcessNoiseVel is the velocity process noise variance (qv).
	// Slightly larger than qp to allow gradual speed changes.
	ProcessNoiseVel float64

	// MeasurementNoise is the GPS observation noise variance (R diagonal).
	// Larger → less trust in GPS readings.
	MeasurementNoise float64

	// SpeedEMA is the exponential moving average factor for speed smoothing.
	// Range (0,1]. Lower = smoother (more inertia).
	SpeedEMA float64

	// VelBlend controls how much the route-guided velocity overrides
	// the Kalman-estimated velocity each step. Range [0,1].
	VelBlend float64

	// RouteSnapAlpha blends the updated position toward the nearest route
	// point each step. Range [0,1]. 0 = no snap, 1 = hard snap.
	RouteSnapAlpha float64
}

// DefaultConfig returns a reasonable default configuration.
func DefaultConfig() Config {
	return Config{
		ProcessNoisePos:  1e-12,
		ProcessNoiseVel:  5e-9,
		MeasurementNoise: 5e-7,
		SpeedEMA:         0.3,
		VelBlend:         0.6,
		RouteSnapAlpha:   0.30,
	}
}

// Filter is a route-guided Kalman filter for 2-D position tracking.
// Call Update() once per GPS observation; internal state is preserved
// between calls so the filter accumulates history automatically.
type Filter struct {
	cfg   Config
	route []Point

	x Vec4 // state  [lon, lat, vlon, vlat]
	P Mat4 // covariance

	F  Mat4
	H  Mat24
	Ht Mat42
	Q  Mat4
	R  Mat2

	smoothSpeed float64
	prev        Point // previous GPS observation
	ready       bool  // true after first Update call
}

// NewFilter creates a filter that will guide predictions along the given route.
func NewFilter(route []Point, cfg Config) *Filter {
	dt := 1.0
	F := Mat4{
		{1, 0, dt, 0},
		{0, 1, 0, dt},
		{0, 0, 1, 0},
		{0, 0, 0, 1},
	}
	H := Mat24{
		{1, 0, 0, 0},
		{0, 1, 0, 0},
	}
	qp, qv := cfg.ProcessNoisePos, cfg.ProcessNoiseVel
	Q := Mat4{
		{qp, 0, 0, 0},
		{0, qp, 0, 0},
		{0, 0, qv, 0},
		{0, 0, 0, qv},
	}
	rr := cfg.MeasurementNoise
	R := Mat2{{rr, 0}, {0, rr}}

	return &Filter{
		cfg:   cfg,
		route: route,
		F:     F,
		H:     H,
		Ht:    transposeH(H),
		Q:     Q,
		R:     R,
	}
}

// Update accepts a single GPS observation, runs one predict-update cycle,
// and returns the inferred position. The filter keeps all internal state
// (position, velocity, covariance, speed EMA) between calls, so you
// simply feed points one at a time in chronological order.
func (f *Filter) Update(gps Point) Point {
	if !f.ready {
		f.x = Vec4{gps.Lon, gps.Lat, 0, 0}
		f.P = Mat4{
			{1e-8, 0, 0, 0},
			{0, 1e-8, 0, 0},
			{0, 0, 1e-7, 0},
			{0, 0, 0, 1e-7},
		}
		f.smoothSpeed = 0
		f.prev = gps
		f.ready = true

		snap, _, _ := SnapToRoute(gps, f.route)
		a := f.cfg.RouteSnapAlpha
		return Point{
			Lon: (1-a)*gps.Lon + a*snap.Lon,
			Lat: (1-a)*gps.Lat + a*snap.Lat,
		}
	}

	result := f.step(gps, f.prev)
	f.prev = gps
	return result
}

// step performs one predict-update cycle.
func (f *Filter) step(cur, prev Point) Point {
	pos := Point{f.x[0], f.x[1]}
	_, dir, _ := SnapToRoute(pos, f.route)

	rawSpeed := Dist(cur, prev)
	f.smoothSpeed = f.cfg.SpeedEMA*rawSpeed + (1-f.cfg.SpeedEMA)*f.smoothSpeed

	vb := f.cfg.VelBlend
	f.x[2] = vb*(dir.Lon*f.smoothSpeed) + (1-vb)*f.x[2]
	f.x[3] = vb*(dir.Lat*f.smoothSpeed) + (1-vb)*f.x[3]

	// Predict
	xPred := mul4v(f.F, f.x)
	PPred := add44(mul44(mul44(f.F, f.P), transpose4(f.F)), f.Q)

	// Update
	z := Vec2{cur.Lon, cur.Lat}
	y := subv2(z, mul24v(f.H, xPred))
	S := add22(mul24_42(mul24_44(f.H, PPred), f.Ht), f.R)
	K := mul42_22(mul44_42(PPred, f.Ht), inv2(S))
	f.x = addv4(xPred, mul42v(K, y))
	f.P = mul44(sub44(eye4(), mul42_24(K, f.H)), PPred)

	// Soft route snap
	a := f.cfg.RouteSnapAlpha
	snap, _, _ := SnapToRoute(Point{f.x[0], f.x[1]}, f.route)
	f.x[0] = (1-a)*f.x[0] + a*snap.Lon
	f.x[1] = (1-a)*f.x[1] + a*snap.Lat

	return Point{f.x[0], f.x[1]}
}
