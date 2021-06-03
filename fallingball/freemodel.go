package fallingball

import (
	"math"

	"github.com/milosgajdos/go-estimate/sim"
	"gonum.org/v1/gonum/diff/fd"
	"gonum.org/v1/gonum/mat"
)

//  physics constants
const (
	// gravity for falling ball model
	g = -10.
	// viscous acceleration coeficient for ball (Drag force over mass)
	ξ = 0.1
)

// FreeModel returns a control theory model for a falling ball represented by
// the equations:
//  dx/dt = v  (change in position equal to velocity variable)
//  dv/dt = g - |v|*v (change in velocity equal to gravity field m)
//
// C matrix is [1 0], which represents observable state. The only observable
// state in this case is position. dt is the step
// taken to advance solution in Propagate().
//
// B is nil, which means there is no input vector. This model is not controllable.
func FreeModel(dt float64) *sim.BaseModel {
	xeq := []float64{0., 0.}
	A := mat.NewDense(2, 2, nil)
	fd.Jacobian(A, SODE, xeq, nil)

	C := mat.NewDense(1, 2, []float64{1.0, 0.0})

	b, err := sim.NewBaseModel(A, nil, C, nil, nil, dt)
	if err != nil {
		panic(err)
	}
	return b
}

// SODE is the system of differential equations that
// represent the falling ball physical system
//
//  g is local gravity field expressed as a scalar
//  ξ is viscous drag coefficient experienced by the ball due to being in a fluid
//
// y is the result of said equations which are written in function of x as follows:
//  y(t) = x'(t) = f(t, x)
// where f is SODE. This system is not controllable as there is no input vector.
//
// In this case we have the following equations:
//  dx/dt = v  (change in position equal to velocity variable)
//  dv/dt = g - |v|*v (change in velocity equal to gravity field m)
func SODE(y, x []float64) {
	y[0] = x[1]
	y[1] = g - ξ*x[1]*math.Abs(x[1])
}
