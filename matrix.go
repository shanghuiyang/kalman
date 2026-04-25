package kalman

type Vec4 [4]float64
type Vec2 [2]float64
type Mat4 [4][4]float64
type Mat42 [4][2]float64
type Mat24 [2][4]float64
type Mat2 [2][2]float64

func eye4() Mat4 {
	return Mat4{
		{1, 0, 0, 0},
		{0, 1, 0, 0},
		{0, 0, 1, 0},
		{0, 0, 0, 1},
	}
}

func mul44(a, b Mat4) Mat4 {
	var c Mat4
	for i := 0; i < 4; i++ {
		for j := 0; j < 4; j++ {
			for k := 0; k < 4; k++ {
				c[i][j] += a[i][k] * b[k][j]
			}
		}
	}
	return c
}

func mul4v(m Mat4, v Vec4) Vec4 {
	var r Vec4
	for i := 0; i < 4; i++ {
		for j := 0; j < 4; j++ {
			r[i] += m[i][j] * v[j]
		}
	}
	return r
}

func mul42v(m Mat42, v Vec2) Vec4 {
	var r Vec4
	for i := 0; i < 4; i++ {
		for j := 0; j < 2; j++ {
			r[i] += m[i][j] * v[j]
		}
	}
	return r
}

func mul24v(m Mat24, v Vec4) Vec2 {
	var r Vec2
	for i := 0; i < 2; i++ {
		for j := 0; j < 4; j++ {
			r[i] += m[i][j] * v[j]
		}
	}
	return r
}

func add44(a, b Mat4) Mat4 {
	var c Mat4
	for i := 0; i < 4; i++ {
		for j := 0; j < 4; j++ {
			c[i][j] = a[i][j] + b[i][j]
		}
	}
	return c
}

func sub44(a, b Mat4) Mat4 {
	var c Mat4
	for i := 0; i < 4; i++ {
		for j := 0; j < 4; j++ {
			c[i][j] = a[i][j] - b[i][j]
		}
	}
	return c
}

func addv4(a, b Vec4) Vec4 {
	return Vec4{a[0] + b[0], a[1] + b[1], a[2] + b[2], a[3] + b[3]}
}

func subv2(a, b Vec2) Vec2 {
	return Vec2{a[0] - b[0], a[1] - b[1]}
}

func mul24_44(h Mat24, p Mat4) Mat24 {
	var r Mat24
	for i := 0; i < 2; i++ {
		for j := 0; j < 4; j++ {
			for k := 0; k < 4; k++ {
				r[i][j] += h[i][k] * p[k][j]
			}
		}
	}
	return r
}

func mul44_42(p Mat4, ht Mat42) Mat42 {
	var r Mat42
	for i := 0; i < 4; i++ {
		for j := 0; j < 2; j++ {
			for k := 0; k < 4; k++ {
				r[i][j] += p[i][k] * ht[k][j]
			}
		}
	}
	return r
}

func mul24_42(a Mat24, b Mat42) Mat2 {
	var r Mat2
	for i := 0; i < 2; i++ {
		for j := 0; j < 2; j++ {
			for k := 0; k < 4; k++ {
				r[i][j] += a[i][k] * b[k][j]
			}
		}
	}
	return r
}

func add22(a, b Mat2) Mat2 {
	return Mat2{
		{a[0][0] + b[0][0], a[0][1] + b[0][1]},
		{a[1][0] + b[1][0], a[1][1] + b[1][1]},
	}
}

func inv2(m Mat2) Mat2 {
	det := m[0][0]*m[1][1] - m[0][1]*m[1][0]
	return Mat2{
		{m[1][1] / det, -m[0][1] / det},
		{-m[1][0] / det, m[0][0] / det},
	}
}

func mul42_22(a Mat42, b Mat2) Mat42 {
	var r Mat42
	for i := 0; i < 4; i++ {
		for j := 0; j < 2; j++ {
			for k := 0; k < 2; k++ {
				r[i][j] += a[i][k] * b[k][j]
			}
		}
	}
	return r
}

func mul42_24(a Mat42, b Mat24) Mat4 {
	var r Mat4
	for i := 0; i < 4; i++ {
		for j := 0; j < 4; j++ {
			for k := 0; k < 2; k++ {
				r[i][j] += a[i][k] * b[k][j]
			}
		}
	}
	return r
}

func transpose4(m Mat4) Mat4 {
	var t Mat4
	for i := 0; i < 4; i++ {
		for j := 0; j < 4; j++ {
			t[i][j] = m[j][i]
		}
	}
	return t
}

func transposeH(h Mat24) Mat42 {
	var t Mat42
	for i := 0; i < 2; i++ {
		for j := 0; j < 4; j++ {
			t[j][i] = h[i][j]
		}
	}
	return t
}
