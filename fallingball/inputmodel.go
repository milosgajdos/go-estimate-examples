package fallingball

import (
	"math"

	"github.com/milosgajdos/go-estimate/sim"
	"gonum.org/v1/gonum/diff/fd"
	"gonum.org/v1/gonum/mat"
)

// InputModel returns a control theory model for a falling ball represented by
// the equations:
//  dx/dt = v  (change in position equal to velocity variable)
//  dv/dt = g - |v|*v + u (change in velocity equal to gravity field m)
// where `u` is a input to keep ball at equilibrium (0,0). dt is the step
// taken to advance solution in Propagate().
//
// C matrix is [1 0], which represents observable state. The only observable
// state in this case is position.
//
// B non-zero which means this model is controllable.
func InputModel(dt float64) *sim.BaseModel {
	xeq := []float64{0., 0.}
	ueq := []float64{-g} // to keep ball upright we must make opposite force of gravity
	// calculate SystemMatrix linearizing around equilibrium
	A := mat.NewDense(2, 2, nil)
	fd.Jacobian(A, SODE, xeq, nil)
	// calculate input matrix from equilibrium
	insode := func(y, u []float64) {
		InputSODE(y, xeq, u)
	}
	B := mat.NewDense(2, 1, nil)
	fd.Jacobian(B, insode, ueq, nil)

	C := mat.NewDense(1, 2, []float64{1.0, 0.0})

	b, err := sim.NewBaseModel(A, B, C, nil, nil, dt)
	if err != nil {
		panic(err)
	}
	return b
}

// SODE is the system of differential equations that
// represent the falling ball physical system
//
//  g is local gravity field expressed as a scalar
//  ξ is viscous drag coefficient expereienced by the ball due to being in a fluid
//
// y is the result of said equations which are written in function of x as follows:
//  y(t) = x'(t) = f(t, x)
// where f is FreeSODE
func InputSODE(y, x, u []float64) {
	y[0] = x[1]
	y[1] = g - ξ*x[1]*math.Abs(x[1]) + u[0]
}
